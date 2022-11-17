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
from abb_robot_client.egm import EGM
from abb_robot_client.rws import RWS
import abb_motion_program_exec as abb
from robotraconteur_abstract_robot import AbstractRobot
import traceback
import time

class ABBRobotImpl(AbstractRobot):
    def __init__(self, robot_info, robot_url):
        super().__init__(robot_info, 6)
        self._robot_url = robot_url
        self._uses_homing = False
        self._has_position_command = True
        self._has_velocity_command = False
        self._update_period = 4e-3
        self.robot_info.robot_capabilities &= self._robot_capabilities["jog_command"] \
             & self._robot_capabilities["position_command"] & self._robot_capabilities["trajectory_command"]
        self._trajectory_error_tol = 1000
        self._egm_client = None

    def _start_robot(self):
        self._egm = EGM()
        super()._start_robot()

    def _send_disable(self, handler):
        raise NotImplementedError()

    def _send_enable(self, handler):
        raise NotImplementedError()

    def _send_reset_errors(self, handler):
        raise NotImplementedError()

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
        if robot_state is not None:
            egm_last_recv = self._stopwatch_ellapsed_s()
            self._operational_mode = self._robot_operational_mode["auto"]
            self._last_joint_state = egm_last_recv
            self._last_endpoint_state = egm_last_recv
            self._last_robot_state = egm_last_recv
            self._enabled = robot_state.motors_on
            self._ready = robot_state.rapid_running
        
            self._joint_position = np.deg2rad(robot_state.joint_angles)            
            self._endpoint_pose = self._node.ArrayToNamedArray(\
                np.concatenate((robot_state.cartesian[1],robot_state.cartesian[0])), self._pose_dtype)
            
        else:
            if self._communication_failure:
                self._joint_position = np.zeros((0,))        
                self._endpoint_pose = np.zeros((0,),dtype=self._pose_dtype)

        super()._run_timestep(now)

        if self._command_mode == self._robot_command_mode["halt"] or \
             self._command_mode == self._robot_command_mode["invalid_state"]:
             self._position_command = None

        if self._egm.egm_addr is not None:
            if self._position_command is None:
                self._egm.send_to_robot(None)
            else:
                self._egm.send_to_robot(np.rad2deg(self._position_command))

    def _close(self):
        if self._egm_client is not None:
            self.egm_client.close()
        return super()._close()

def main():
    parser = argparse.ArgumentParser(description="ABB robot driver service for Robot Raconteur")
    parser.add_argument("--robot-info-file", type=argparse.FileType('r'),default=None,required=True,help="Robot info file (required)")
    parser.add_argument("--robot-url", type=str,default=None,help="Robot Web Services URL")
    parser.add_argument("--robot-name", type=str,default=None,help="Optional device name override")
    parser.add_argument("--wait-signal",action='store_const',const=True,default=False, help="wait for SIGTERM orSIGINT (Linux only)")

    args, _ = parser.parse_known_args()

    rr_args = ["--robotraconteur-jumbo-message=true"] + sys.argv

    RRC.RegisterStdRobDefServiceTypes(RRN)

    with args.robot_info_file:
        robot_info_text = args.robot_info_file.read()

    info_loader = InfoFileLoader(RRN)
    robot_info, robot_ident_fd = info_loader.LoadInfoFileFromString(robot_info_text, "com.robotraconteur.robotics.robot.RobotInfo", "device")

    attributes_util = AttributesUtil(RRN)
    robot_attributes = attributes_util.GetDefaultServiceAttributesFromDeviceInfo(robot_info.device_info)

    robot = ABBRobotImpl(robot_info, args.robot_url)
    try:

        robot._start_robot()
        with RR.ServerNodeSetup("com.robotraconteur.robotics.robot.abb",59925,argv=rr_args):

            service_ctx = RRN.RegisterService("robot","com.robotraconteur.robotics.robot.Robot",robot)
            service_ctx.SetServiceAttributes(robot_attributes)

            if args.wait_signal:  
                #Wait for shutdown signal if running in service mode          
                print("Press Ctrl-C to quit...")
                import signal
                signal.sigwait([signal.SIGTERM,signal.SIGINT])
            else:
                #Wait for the user to shutdown the service
                if (sys.version_info > (3, 0)):
                    input("Server started, press enter to quit...")
                else:
                    raw_input("Server started, press enter to quit...")
    finally:
        robot._close()