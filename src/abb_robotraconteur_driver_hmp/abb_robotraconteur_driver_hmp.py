import RobotRaconteur as RR
RRN = RR.RobotRaconteurNode.s
import RobotRaconteurCompanion as RRC
import argparse
import sys
import platform
import threading
import numpy as np
from RobotRaconteurCompanion.Util.InfoFileLoader import InfoFileLoader
from RobotRaconteurCompanion.Util.DateTimeUtil import DateTimeUtil
from RobotRaconteurCompanion.Util.SensorDataUtil import SensorDataUtil
from RobotRaconteurCompanion.Util.AttributesUtil import AttributesUtil
from RobotRaconteurCompanion.Util.RobDef import register_service_types_from_resources
from abb_robot_client.egm import EGM
from abb_robot_client.rws import RWS
import abb_motion_program_exec as abb
from robotraconteur_abstract_robot import AbstractRobot
import traceback
import time
from . import _rws as rws
from ._joint_control_req import JointControlReq
from contextlib import suppress
from ._motion_program import MotionExecImpl, RobotRecordingGen
from aioconsole import ainput
import asyncio

class ABBRobotImpl(AbstractRobot):
    def __init__(self, robot_info, robot_url):
        super().__init__(robot_info, 6)
        self._robot_url = robot_url
        self._uses_homing = False
        self._has_position_command = True
        self._has_velocity_command = False
        self._update_period = 4e-3
        self._base_set_controller_state = False
        self._base_set_operational_mode = False
        self.robot_info.robot_capabilities &= self._robot_capabilities["jog_command"] \
             & self._robot_capabilities["position_command"] & self._robot_capabilities["trajectory_command"] \
             & self._robot_capabilities["software_enable"] & self._robot_capabilities["software_reset_errors"]
        self._trajectory_error_tol = 1000
        self._egm_client = None
        self._missed_egm = 10000
        self._rws = rws.ABBRobotRWSImpl(self._robot_url)
        self._joint_control_req = None
        self._joint_control_req_last_attempt = 0
        self._motion_exec_impl = MotionExecImpl(self)

        self._motion_program_recordings = dict()

        self._motion_program_consts = self._node.GetConstants("experimental.robotics.motion_program")
        self._motion_program_flags = self._motion_program_consts["MotionProgramRobotStateFlags"]

        self._motion_program_robot_state_type = self._node.GetStructureType(
            "experimental.robotics.motion_program.MotionProgramRobotState")

        self.motion_program_robot_info = self._motion_exec_impl.get_motion_program_robot_info(self.robot_info)

    def RRServiceObjectInit(self, context, service_path):
        super().RRServiceObjectInit(context, service_path)

        self.egm_joint_command.InValueChanged += self._egm_joint_command_invalue_changed
        self.egm_pose_command.InValueChanged += self._egm_pose_command_invalue_changed
        self.egm_path_correction_command.InValueChanged += self._egm_correction_command_invalue_changed

        self._broadcast_downsampler.AddWireBroadcaster(self.motion_program_robot_state)

    def _start_robot(self):
        self._egm = EGM()
        self._rws.start()
        self._joint_control_req = JointControlReq(self, self._rws)
        self._joint_control_req.start()
        self._missed_egm = 10000
        super()._start_robot()
        time.sleep(0.5)

    def _send_disable(self, handler):
        self._rws.send_disable()
        self._node.PostToThreadPool(lambda: handler(None))

    def _send_enable(self, handler):
        self._rws.send_enable()
        self._node.PostToThreadPool(lambda: handler(None))

    def _send_reset_errors(self, handler):
        self._rws.reset_errors()
        with suppress(Exception):
            self.command_mode = 0
        self._node.PostToThreadPool(lambda: handler(None))

    def _send_robot_command(self, now, joint_pos_cmd, joint_vel_cmd):
        if joint_pos_cmd is not None:
            self._position_command = joint_pos_cmd
        else:
            self._position_command = None
        
    def _run_timestep(self, now):
        res = True
        robot_state = None
        while res:
            res, robot_state1 = self._egm.receive_from_robot()
            if res:                
                robot_state = robot_state1
                self._missed_egm = 0
        if robot_state is None:
            self._missed_egm += 1
        joint_ctrl_required = self._command_mode in (0,1,2,3)
        joint_ctrl_error = self._joint_control_req.error_obj is not None
        joint_ctrl_ready = self._joint_control_req.ready
        
        self._joint_control_req.loop_set_enabled(joint_ctrl_required)

        if robot_state is not None:
            egm_last_recv = self._stopwatch_ellapsed_s()
            self._last_joint_state = egm_last_recv
            self._last_endpoint_state = egm_last_recv
            self._last_robot_state = egm_last_recv
            self._enabled = robot_state.motors_on
            self._ready = robot_state.rapid_running
            if joint_ctrl_required:
                self._ready &= joint_ctrl_ready
        
            self._joint_position = np.deg2rad(robot_state.joint_angles)            
            self._endpoint_pose = self._node.ArrayToNamedArray(\
                np.concatenate((robot_state.cartesian[1],robot_state.cartesian[0]*1e-3)), self._pose_dtype)
            
        else:
            if self._communication_failure:
                self._joint_position = np.zeros((0,))        
                self._endpoint_pose = np.zeros((0,),dtype=self._pose_dtype)

        if self._rws.connection_status == rws.ConnectionStatus.connected:
            self._controller_state = self._rws.controller_state
            self._stopped = self._controller_state == self._robot_controller_state["emergency_stop"]
            self._operational_mode = self._rws.controller_opmode
            self._error = self._rws.exec_error or self._stopped  # or joint_ctrl_error
        else:
            self._controller_state = self._robot_controller_state["undefined"]
            self._operational_mode = self._robot_operational_mode["undefined"]
            self._stopped = False
            self._error = joint_ctrl_error

        self._rws.egm_communication_failure = self._missed_egm > 500

        if self._error:
            self._ready = False

        super()._run_timestep(now)

        if self._command_mode == self._robot_command_mode["halt"] or \
             self._command_mode == self._robot_command_mode["invalid_state"]:
             self._position_command = None

        if self._egm.egm_addr is not None and self._command_mode != 6:
            if self._position_command is None:
                self._egm.send_to_robot(None)
            else:
                self._egm.send_to_robot(np.rad2deg(self._position_command))

    def _verify_communication(self, now):

        res = super()._verify_communication(now)
        if self._rws.connection_status != rws.ConnectionStatus.connected:
            self._communication_failure = True
            res = False
        return res

    def _close(self):
        if self._egm_client is not None:
            self.egm_client.close()
        with suppress(Exception):
            self._rws.send_stop_all()
        return super()._close()

    def execute_motion_program(self, program, queue):
        assert not queue, "Queuing motion programs not supported"
        assert self._command_mode == 6, "Invalid mode for motion program"

        return self._motion_exec_impl.execute_motion_program(program)

    def execute_motion_program_record(self, program, queue):
        assert not queue, "Queuing motion programs not supported"
        assert self._command_mode == 6, "Invalid mode for motion program"

        return self._motion_exec_impl.execute_motion_program(program, True)

    def preempt_motion_program(self, program, preempt_number, preempt_cmdnum):
        assert self._command_mode == 6, "Invalid mode for motion program"

        return self._motion_exec_impl.preempt_motion_program(program, preempt_number, preempt_cmdnum)

    def _egm_joint_command_invalue_changed(self, value, ts, ep):
        try:
            if self._command_mode == 6:
                self._egm.send_to_robot(value.joints)
        except:
            traceback.print_exc()

    def _egm_pose_command_invalue_changed(self, value, ts, ep):
        # print(f"egm_pose_command: {value.cartesian}")
        try:
            if self._command_mode == 6:
                c = self._node.NamedArrayToArray(value.cartesian)[0]
                self._egm.send_to_robot_cart(np.array(c[4:7])*1e3, c[0:4])
        except:
            traceback.print_exc()

    def _egm_correction_command_invalue_changed(self, value, ts, ep):
        # print(f"egm_correction_command: {value.pos}, {value.age}")
        try:
            if self._command_mode == 6:
                p = self._node.NamedArrayToArray(value.pos)[0]*1e3
                self._egm.send_to_robot_path_corr(p, value.age)
        except:
            traceback.print_exc()


    # Overrides for hybrid control
    @AbstractRobot.command_mode.setter
    def command_mode(self, value):
        self._lock.acquire()
        locked = True
        try:
            if self._command_mode == self._robot_command_mode["halt"] and value == 6 and not self._error and \
                self._rws.connection_status == rws.ConnectionStatus.connected:

                self._command_mode = 6
                time.sleep(0.02)
                self._joint_control_req.disable(timeout=0.5)
                return
            if self._command_mode == 6 and value == self._robot_command_mode["halt"]:
                self._command_mode = self._robot_command_mode["halt"]
                time.sleep(0.02)
                self._joint_control_req.enable(timeout=0.5)
                return
        finally:
            if locked:
                self._lock.release()
        AbstractRobot.command_mode.fset(self, value)

    def read_recording(self, recording_handle):
        robot_rec_np = self._motion_program_recordings.pop(recording_handle)
        return RobotRecordingGen(self, robot_rec_np)

    def clear_recordings(self):
        self._motion_program_recordings.clear()

    def _fill_state_flags(self, now):
        flags = super()._fill_state_flags(now)
        if self._command_mode == 6:
            flags |= self._motion_program_flags["motion_program_mode_enabled"]
        if self._rws.motion_program_state in (rws.MotionProgramState.running, 
            rws.MotionProgramState.running_egm_joint_control,
            rws.MotionProgramState.running_egm_pose_control,
            rws.MotionProgramState.running_egm_path_corr):
            flags |= self._motion_program_flags["motion_program_running"]
        return flags

    def _fill_mp_state(self, now, rr_advanced_robot_state):
        mp_state = self._motion_program_robot_state_type()
        mp_state.ts = rr_advanced_robot_state.ts
        mp_state.seqno = rr_advanced_robot_state.seqno
        mp_state.operational_mode = rr_advanced_robot_state.operational_mode
        mp_state.controller_state = rr_advanced_robot_state.controller_state
        mp_state.motion_program_robot_state_flags = rr_advanced_robot_state.robot_state_flags
        mp_state.current_command = self._rws.exec_current_cmd_num
        mp_state.queued_command = self._rws.exec_queued_cmd_num
        mp_state.current_preempt = self._rws.exec_current_preempt_num
        return mp_state

    def _send_states(self, now, rr_robot_state, rr_advanced_robot_state, rr_state_sensor_data):
        
        super()._send_states(now, rr_robot_state, rr_advanced_robot_state, rr_state_sensor_data)

        mp_state = self._fill_mp_state(now, rr_advanced_robot_state)
        self.motion_program_robot_state.OutValue = mp_state

    def enable_motion_program_mode(self):
        self.command_mode = 6

    def disable_motion_program_mode(self):
        self.command_mode = 0

    def tool_attached(self, chain, tool):
        assert not (self._command_mode in (2,3,4)), "Tool cannot be changed in current mode"
        super().tool_attached(chain, tool)
        self._joint_control_req.request_reload()
    
    def tool_detached(self, chain, tool_name):
        assert not (self._command_mode in (2,3,4)), "Tool cannot be changed in current mode"
        super().tool_detached(chain, tool_name)
        self._joint_control_req.request_reload()

    def payload_attached(self, chain, payload, payload_pose):
        assert not (self._command_mode in (2,3,4)), "Payload cannot be changed in current mode"
        super().payload_attached(chain, payload, payload_pose)
        self._joint_control_req.request_reload()

    def payload_detached(self, chain, payload_name):
        assert not (self._command_mode in (2,3,4)), "Payload cannot be changed in current mode"
        super().payload_detached(chain, payload_name)
        self._joint_control_req.request_reload()

    def _calc_endpoint_pose(self, chain):
        return self._endpoint_pose[chain]

    def _calc_endpoint_vel(self, chain):
        return self._endpoint_vel[chain]

async def amain():

    parser = argparse.ArgumentParser(description="ABB robot driver service for Robot Raconteur")
    parser.add_argument("--robot-info-file", type=argparse.FileType('r'),default=None,required=True,help="Robot info file (required)")
    parser.add_argument("--robot-url", type=str,default="http://127.0.0.1:80",help="Robot Web Services URL")
    parser.add_argument("--robot-name", type=str,default=None,help="Optional device name override")
    parser.add_argument("--wait-signal",action='store_const',const=True,default=False, help="wait for SIGTERM orSIGINT (Linux only)")

    args, _ = parser.parse_known_args()

    rr_args = ["--robotraconteur-jumbo-message=true"] + sys.argv

    RRC.RegisterStdRobDefServiceTypes(RRN)
    register_service_types_from_resources(RRN, __package__, ["experimental.robotics.motion_program", 
        "experimental.abb_robot", "experimental.abb_robot.motion_program"])

    with args.robot_info_file:
        robot_info_text = args.robot_info_file.read()

    info_loader = InfoFileLoader(RRN)
    robot_info, robot_ident_fd = info_loader.LoadInfoFileFromString(robot_info_text, "com.robotraconteur.robotics.robot.RobotInfo", "device")

    attributes_util = AttributesUtil(RRN)
    robot_attributes = attributes_util.GetDefaultServiceAttributesFromDeviceInfo(robot_info.device_info)

    robot = ABBRobotImpl(robot_info, args.robot_url)
    try:

        robot._start_robot()
        await asyncio.sleep(0.5)
        with RR.ServerNodeSetup("experimental.abb_robot.robot",59925,argv=rr_args):

            service_ctx = RRN.RegisterService("robot","experimental.abb_robot.ABBRobot",robot)
            service_ctx.SetServiceAttributes(robot_attributes)
            service_ctx.AddExtraImport("experimental.abb_robot.motion_program")

            if args.wait_signal:
                #Wait for shutdown signal if running in service mode
                exit_evt = asyncio.Event()    
                print("Press Ctrl-C to quit...")
                import signal
                signal.signal(signal.SIGTERM, lambda: exit_evt.set())
                signal.signal(signal.SIGINT, lambda: exit_evt.set())
                await exit_evt.wait()
            else:                
                await ainput("Server started, press enter to quit...")
            robot._close()
    except:
        with suppress(Exception):
            robot._close()
        raise

def main():
    asyncio.run(amain())
