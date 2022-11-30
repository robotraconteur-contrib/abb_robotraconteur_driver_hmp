
import RobotRaconteur as RR
import numpy as np
import abb_motion_program_exec as abb_exec
from RobotRaconteur.RobotRaconteurPythonUtil import NamedArrayToArray
import threading
import traceback
import time
from . import _rws as rws
import asyncio
from contextlib import suppress

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
        
        self._closed = False

        self._mp_lock = asyncio.Lock()
        self._mp_task = None
        self._mp_states = None

        

    
    def AsyncNext(self, handler):
        with self._lock:
            if self._closed:
                raise RR.StopIterationException("")
            
        if self._status == self._action_status_code["queued"]:
            async def _start_mp():
                try:
                    mp_task, mp_states = self._rws.execute_motion_program(self._motion_program, 
                        enable_motion_logging = self._save_log)
                    
                    with self._lock:
                        self._mp_task = mp_task
                        self._mp_states = mp_states
                        if self._closed:
                            self._mp_task.cancel()
                    self._status = self._action_status_code["running"]
                except BaseException as e:
                    traceback.print_exc()
                    err = e
                    self._closed = True
                    self._status = self._action_status_code["error"]
                    handler(None, err)
                else:
                    ret = self._mp_status()
                    ret.current_command = -1
                    ret.queued_command = -1
                    ret.action_status = self._status
                    handler(ret, None)
            asyncio.run_coroutine_threadsafe(_start_mp(), self._rws.loop)
            return

        if self._status == self._action_status_code["running"]:
            async def _run_mp():
                ret = self._mp_status()
                ret.current_command = -1
                ret.queued_command = -1
                try:
                    state, data = await self._mp_states.get()
                    if state in (rws.MotionProgramState.running, rws.MotionProgramState.running_egm_joint_control,
                        rws.MotionProgramState.running_egm_path_corr, rws.MotionProgramState.running_egm_pose_control):
                        ret.current_command, ret.queued_command = data
                    elif state in (rws.MotionProgramState.starting, rws.MotionProgramState.stopping, 
                        rws.MotionProgramState.stop_requested, rws.MotionProgramState.stopping_egm):
                        pass
                    elif state == rws.MotionProgramState.complete:
                        self._closed = True
                        self._status = self._action_status_code["complete"]
                        # TODO: store recording result
                    elif state == rws.MotionProgramState.cancelled:
                        raise RR.OperationAbortedException("Motion program cancelled")
                    elif state == rws.MotionProgramState.error:
                        raise data
                    else:
                        raise Exception(f"Unknown motion program state {state}")
                except BaseException as e:
                    traceback.print_exc()
                    self._status == self._action_status_code["error"]
                    err = e
                    self._closed = True
                    handler(None, err)
                else:
                    ret.action_status = self._status
                    handler(ret, None)
            asyncio.run_coroutine_threadsafe(_run_mp(), self._rws.loop)
            return
        raise RR.StopIterationException("")

                
    def Close(self):
        self._closed = True
        with suppress(Exception):
            self._rws.loop.call_soon_threadsafe(lambda: self._mp_task.cancel())

    def Abort(self):
        self._closed = True
        with suppress(Exception):
            self._rws.loop.call_soon_threadsafe(lambda: self._mp_task.cancel())

