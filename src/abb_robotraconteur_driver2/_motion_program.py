
import RobotRaconteur as RR
import numpy as np
import abb_motion_program_exec as abb_exec
from RobotRaconteur.RobotRaconteurPythonUtil import NamedArrayToArray
import threading
import traceback
import time
from . import _rws as rws

def rr_pose_to_abb(rr_pose):
    a = NamedArrayToArray(rr_pose)
    return abb_exec.pose(a[0][4:7]*1000.0,a[0][0:4])

def rr_tool_to_abb(rr_tool_info):
    tcp = rr_pose_to_abb(rr_tool_info.tcp)
    mass = rr_tool_info.inertia[0]["m"]
    com = NamedArrayToArray(rr_tool_info.inertia[0]["com"])
    #TODO: figure out aom term in loaddata
    aom = np.array([1,0,0,0],dtype=np.float64)
    ix = rr_tool_info.inertia[0]["ixx"]
    iy = rr_tool_info.inertia[0]["iyy"]
    iz = rr_tool_info.inertia[0]["izz"]
    ld = abb_exec.loaddata(mass, com, aom, ix, iy, iz)
    return abb_exec.tooldata(True,tcp,ld)

def rr_zone_to_abb(rr_fine_point,rr_blend_radius):
    r = rr_blend_radius * 1000.0
    #TODO use "extended" fields for external axes
    return abb_exec.zonedata(rr_fine_point,r,r,r,r,r,r)

def rr_speed_to_abb(rr_velocity):
    #TODO use "extended" for angular velocity and external axes?
    return abb_exec.speeddata(rr_velocity*1000.0, rr_velocity*10000.0, 1000, 1000)

def rr_joints_to_abb(rr_joints, rr_joint_units):
    #TODO: joint units
    #TODO: use "extended" for external axes
    return abb_exec.jointtarget(np.rad2deg(rr_joints), [6e5]*6)

def rr_robot_pose_to_abb(rr_robot_pose):
    #TODO: joint units
    #TODO: use "extended" for external axes
    p = rr_pose_to_abb(rr_robot_pose.tcp_pose)
    cd = abb_exec.confdata(
        np.floor(rr_robot_pose.joint_position_seed[0]/(np.pi/2)),
        np.floor(rr_robot_pose.joint_position_seed[3]/(np.pi/2)),
        np.floor(rr_robot_pose.joint_position_seed[5]/(np.pi/2)),
        0
    )
    return abb_exec.robtarget(p.trans,p.rot,cd, [6e5]*6)

def rr_motion_program_to_abb(rr_mp):
    tool_cmd = rr_mp.motion_program_commands[0]
    assert tool_cmd.datatype.endswith("experimental.robotics.motion_program.SetToolCommand"), "First command for ABB must be SetTool"
    abb_tool = rr_tool_to_abb(tool_cmd.data.tool_info)

    mp = abb_exec.MotionProgram(tool=abb_tool)
    for cmd in rr_mp.motion_program_commands[1:]:
        if cmd.datatype.endswith("experimental.robotics.motion_program.MoveAbsJCommand"):
            zd = rr_zone_to_abb(cmd.data.fine_point,cmd.data.blend_radius)
            sd = rr_speed_to_abb(cmd.data.tcp_velocity)
            jt = rr_joints_to_abb(cmd.data.joint_position, cmd.data.joint_units)
            mp.MoveAbsJ(jt, sd, zd)
        elif cmd.datatype.endswith("experimental.robotics.motion_program.MoveJCommand"):
            zd = rr_zone_to_abb(cmd.data.fine_point,cmd.data.blend_radius)
            sd = rr_speed_to_abb(cmd.data.tcp_velocity)
            rt = rr_robot_pose_to_abb(cmd.data.tcp_pose)
            mp.MoveJ(rt, sd, zd)
        elif cmd.datatype.endswith("experimental.robotics.motion_program.MoveLCommand"):
            zd = rr_zone_to_abb(cmd.data.fine_point,cmd.data.blend_radius)
            sd = rr_speed_to_abb(cmd.data.tcp_velocity)
            rt = rr_robot_pose_to_abb(cmd.data.tcp_pose)
            mp.MoveL(rt, sd, zd)
        elif cmd.datatype.endswith("experimental.robotics.motion_program.MoveCCommand"):
            zd = rr_zone_to_abb(cmd.data.fine_point,cmd.data.blend_radius)
            sd = rr_speed_to_abb(cmd.data.tcp_velocity)
            rt = rr_robot_pose_to_abb(cmd.data.tcp_pose)
            rt2 = rr_robot_pose_to_abb(cmd.data.tcp_via_pose)
            mp.MoveC(rt2, rt,  sd, zd)
        elif cmd.datatype.endswith("experimental.robotics.motion_program.WaitTimeCommand"):
            mp.WaitTime(cmd.data.time)
        else:
            assert False, f"Invalid motion program command type \"{cmd.datatype}\""
    
    return mp




class MotionExecImpl:
    def __init__(self, abb_robot_impl):
        self.abb_robot_impl = abb_robot_impl
        self.rws = abb_robot_impl._rws
        self.node = abb_robot_impl._node

    def execute_motion_program(self, program):

        abb_program = rr_motion_program_to_abb(program)

        gen = ExecuteMotionProgramGen(self, self.rws, abb_program)

        return gen

    def run_timestep(self, now):
        pass


class ExecuteMotionProgramGen:

    def __init__(self, parent, rws, motion_program, save_log = False):
        self._node = parent.node
        self._parent = parent
        self._rws = rws
        self._motion_program = motion_program
        self._action_status_code = self._node.GetConstants("com.robotraconteur.action")["ActionStatusCode"]
        self._status = self._action_status_code["queued"]
        self._mp_status = self._node.GetStructureType("experimental.robotics.motion_program.MotionProgramStatus")
        self._log_handle = 0
        self._save_log = save_log
        self._lock = threading.Lock()
        self._next_handler = None
        self._last_next = 0
        self._exec_exp = None
        self._req = None
        self._closed = False

        self._exec_current_cmd_num = -1
        self._sent_current_cmd_num = -1
        self._exec_queued_cmd_num = -1
        self._sent_queued_cmd_num = -1

    def _default_ret(self):
        ret = self._mp_status()
        ret.current_command = -1
        ret.queued_command = -1
        ret.action_status = self._status
        return ret

    def _call_next_handler(self, ret):
        try:
            if self._next_handler is not None:
                h = self._next_handler
                self._next_handler = None
                h(ret, None)
                self._last_next = time.perf_counter()
        except:
            traceback.print_exc()


    def _call_next_handler_err(self, err):
        self._status = self._action_status_code["error"]
        self._exec_exp = err
        try:
            if self._next_handler is not None:
                h = self._next_handler
                self._next_handler = None
                h(None, err)
                self._last_next = time.perf_counter()
        except:
            traceback.print_exc()
        try:
            if self._req is not None:
                self._rws.stop_motion_program_nolock(self._req)
        except:
            pass

    def _do_next_update_ret(self, state, param):
        
        if state == rws.MotionProgramState.complete:
            self._status = self._action_status_code["complete"]
            ret = self._default_ret()
            self._call_next_handler(ret)
            self._closed = True
            return
        if state == rws.MotionProgramState.error:
            self._call_next_handler_err(param)
            return
        if state in (rws.MotionProgramState.running, rws.MotionProgramState.running_egm_joint_control,
            rws.MotionProgramState.running_egm_path_corr, rws.MotionProgramState.running_egm_pose_control):

            self._status = self._action_status_code["running"]

            send = False
            if param is not None:
                self._exec_current_cmd_num, self._exec_queued_cmd_num = param
                if self._exec_current_cmd_num > self._sent_current_cmd_num or \
                    self._exec_queued_cmd_num > self._sent_queued_cmd_num:

                    self._sent_current_cmd_num = self._exec_current_cmd_num
                    self._sent_queued_cmd_num = self._exec_queued_cmd_num
                    send = True
            
            if time.perf_counter() - self._last_next > 2:
                send = True

            if send:
                ret = self._default_ret()
                ret.current_command = self._sent_current_cmd_num
                ret.queued_command = self._sent_queued_cmd_num
                self._call_next_handler(ret)

    def _exec_ev_handler(self, req, state, param = None):
        with self._lock:
            try:
                assert req == self._req
                self._do_next_update_ret(state, param)
            except Exception as e:
                self._call_next_handler_err(self, e)


    def AsyncNext(self, handler):
        with self._lock:
            if self._closed:
                raise RR.StopIterationException("")
            try:
                if self._exec_exp is not None:
                    raise self._exec_exp
                ret = self._mp_status()
                ret.current_command = -1
                ret.queued_command = -1
                if self._status == self._action_status_code["queued"]:
                    self._req = self._rws.execute_motion_program(self._motion_program, self._exec_ev_handler, 
                        enable_motion_logging = False)
                    self._status = self._action_status_code["running"]
                    ret.action_status = self._status
                    handler(ret, None)
                    return
                if self._status == self._action_status_code["complete"]:
                    ret.action_status = self._status
                    self._closed= True
                    handler(ret, None)
                    return
                if self._status == self._action_status_code["running"]:
                    if self._exec_current_cmd_num > self._sent_current_cmd_num or \
                        self._exec_queued_cmd_num > self._sent_queued_cmd_num:

                        self._sent_current_cmd_num = self._exec_current_cmd_num
                        self._sent_queued_cmd_num = self._exec_queued_cmd_num
                        ret.current_command = self._sent_current_cmd_num
                        ret.queued_command = self._sent_queued_cmd_num
                        handler(ret)
                        return
                    else:
                        self._next_handler = handler
                        self._last_next = time.perf_counter()
                        return
            except Exception as e:
                self._action_status_code = self._action_status_code["error"]
                self._exec_exp = e
                try:
                    if self._req is not None:
                        self._rws.stop_motion_program(self._req)
                except:
                    pass
                raise
                

        # self._wait_evt.wait(timeout=1)
        # if self._thread.is_alive():
        #     ret.action_status = self._action_status_code["running"]
        #     return ret
        # else:
        #     if self._log_handle != 0:
        #         self._status = self._action_status_code["complete"]
        #         ret.action_status = self._status
        #         ret.log_handle = self._log_handle
        #         self._log_handle = 0
        #         return ret
        #     if self._thread_exp:
        #         raise self._thread_exp
        #     raise RR.StopIterationException()

    def Close(self):
        self._closed = True

    def Abort(self):
        if self._status == self._action_status_code["queued"] or self._status == self._action_status_code["running"]:
            self._rws.stop_motion_program(self._req)

    def _run(self):
        try:
            print("Start Motion Program!")
            robot_log_csv = self._abb_client.execute_motion_program(self._motion_program)
            # if self._save_log:
            #     robot_log_io = io.StringIO(robot_log_csv.decode("ascii"))
            #     robot_log_io.seek(0)
            #     robot_log_np = np.genfromtxt(robot_log_io, dtype=np.float64, skip_header=1, delimiter=",")
            #     log_handle = random.randint(0,0xFFFFFFF)
            #     self._parent._logs[log_handle] = robot_log_np
            #     self._log_handle = log_handle
            print("Motion Program Complete!")
        except BaseException as e:
            self._thread_exp = e
            traceback.print_exc()
        self._wait_evt.set()
