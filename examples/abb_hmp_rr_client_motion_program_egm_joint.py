# Example Robot Raconteur client using EGM joint position control with motion program commands
#
# This example demonstrates using EGM position control with a motion program. The ABB hmp driver provides the
# "EGMJointTargetConfigCommand" and "EGMRunJointCommand" commands. These commands allow the robot to be set up
# to use EGM joint target position control, and then to run the EGM position control. The Robot Raconteur wire
# "egm_joint_command" passes the provided command directly to the robot EGM. When done with EGM, call Abort()
# on the motion program generator to stop the EGM.

# This example is quite verbose because it builds up each of the command structures to send to the driver. A more
# practical use cases would put most of these functions into a utility module.

import threading
from RobotRaconteur.Client import *
import time
import numpy as np
from contextlib import suppress
from RobotRaconteurCompanion.Util.GeometryUtil import GeometryUtil

c = RRN.ConnectService('rr+tcp://localhost:59925?service=robot')

c.reset_errors()

time.sleep(0.01)

robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", c)
abb_robot_const = RRN.GetConstants("experimental.abb_robot", c)
abb_robot_mp_const = RRN.GetConstants("experimental.abb_robot.motion_program", c)

halt_mode = robot_const["RobotCommandMode"]["halt"]
motion_program_mode = abb_robot_const["ABBRobotCommandMode"]["motion_program"]
cir_path_mode_switch = abb_robot_mp_const["CirPathModeSwitch"]

moveabsj_type = RRN.GetStructureType("experimental.robotics.motion_program.MoveAbsJCommand",c)
egm_config_jt_type = RRN.GetStructureType("experimental.abb_robot.motion_program.EGMJointTargetConfigCommand",c)
egm_minmax_type = RRN.GetStructureType("experimental.abb_robot.motion_program.egm_minmax",c)
motionprogram_type = RRN.GetStructureType("experimental.robotics.motion_program.MotionProgram",c)
run_egm_type = RRN.GetStructureType("experimental.abb_robot.motion_program.EGMRunJointCommand",c)
egm_jt_type = RRN.GetStructureType("experimental.abb_robot.motion_program.EGMJointTarget",c)

c.command_mode = halt_mode
time.sleep(0.01)
c.command_mode = motion_program_mode

mp_cmds = []

def moveabsj(j,velocity,blend_radius,fine_point):
    cmd = moveabsj_type()
    cmd.joint_position = j
    cmd.tcp_velocity = velocity
    cmd.blend_radius = blend_radius
    cmd.fine_point = fine_point
    return RR.VarValue(cmd,"experimental.robotics.motion_program.MoveAbsJCommand")

mm = egm_minmax_type()
mm.min = -1e-3
mm.max = 1e-3

egm_config_jt = egm_config_jt_type()
egm_config_jt.J1 = mm
egm_config_jt.J2 = mm
egm_config_jt.J3 = mm
egm_config_jt.J4 = mm
egm_config_jt.J5 = mm
egm_config_jt.J6 = mm
egm_config_jt.max_position_deviation = 1000
egm_config_jt.max_speed_deviation = 1000

setup_cmds = [RR.VarValue(egm_config_jt,"experimental.abb_robot.motion_program.EGMJointTargetConfigCommand")]

mp_cmds.append(moveabsj(np.zeros((6,)),1,0.1,True))

run_egm = run_egm_type()
run_egm.cond_time = 10
run_egm.ramp_in_time = 0.05
run_egm.ramp_out_time = 0.05
mp_cmds.append(RR.VarValue(run_egm,"experimental.abb_robot.motion_program.EGMRunJointCommand"))

mp = motionprogram_type()

mp.motion_setup_commands = setup_cmds
mp.motion_program_commands = mp_cmds

robot_state_w = c.robot_state.Connect()
robot_state_w.WaitInValueValid()
egm_cmd_w = c.egm_joint_command.Connect()

mp_gen = c.execute_motion_program(mp, False)
res = None

t1 = time.perf_counter()

egm_keep_going = True

def egm_send_thread():
    t1 = time.perf_counter()
    seqno = 1
    while egm_keep_going:
        t2 = time.perf_counter()
        st = robot_state_w.InValue
        jt = egm_jt_type()        
        seqno += 1
        jt.seqno = seqno
        jt.state_seqno = st.seqno
        jt.joints = np.ones((6,))*np.sin(t2-t1)*5
        egm_cmd_w.OutValue = jt
        time.sleep(0.01)

thread = threading.Thread(target=egm_send_thread)
thread.daemon = True
thread.start()

with suppress(RR.StopIterationException,RR.OperationAbortedException):
    while True:
        res = mp_gen.Next()
        print(f"res.status: {res.action_status}, res.cmd: {res.current_command}, res.queued: {res.queued_command}")

        t2 = time.perf_counter()
        if t2 -t1 > 10:
            mp_gen.Abort()

egm_keep_going = False
        
print("Done!")