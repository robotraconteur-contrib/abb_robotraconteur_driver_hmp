# Example Robot Raconteur client using EGM pose control with motion program commands
#
# This example demonstrates using EGM pose control with a motion program. The ABB hmp driver provides the
# "EGMPoseTargetConfigCommand" and "EGMRunPoseCommand" commands. These commands allow the robot to be set up
# to use EGM pose target position control, and then to run the EGM pose control. The Robot Raconteur wire
# "egm_pose_command" passes the provided command directly to the robot EGM. When done with EGM, call Abort()
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

geom_util = GeometryUtil(client_obj = c)

c.reset_errors()

time.sleep(0.01)

robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", c)
abb_robot_const = RRN.GetConstants("experimental.abb_robot", c)
abb_robot_mp_const = RRN.GetConstants("experimental.abb_robot.motion_program", c)

halt_mode = robot_const["RobotCommandMode"]["halt"]
motion_program_mode = abb_robot_const["ABBRobotCommandMode"]["motion_program"]
cir_path_mode_switch = abb_robot_mp_const["CirPathModeSwitch"]
egmframetype = abb_robot_mp_const["egmframetype"]

robot_pose_type = RRN.GetStructureType("experimental.robotics.motion_program.RobotPose",c)
movej_type = RRN.GetStructureType("experimental.robotics.motion_program.MoveJCommand",c)
egm_config_pt_type = RRN.GetStructureType("experimental.abb_robot.motion_program.EGMPoseTargetConfigCommand",c)
egm_minmax_type = RRN.GetStructureType("experimental.abb_robot.motion_program.egm_minmax",c)
motionprogram_type = RRN.GetStructureType("experimental.robotics.motion_program.MotionProgram",c)
run_egm_type = RRN.GetStructureType("experimental.abb_robot.motion_program.EGMRunPoseCommand",c)
egm_pt_type = RRN.GetStructureType("experimental.abb_robot.motion_program.EGMPoseTarget",c)
pose_dtype = RRN.GetNamedArrayDType("com.robotraconteur.geometry.Pose", c)

c.command_mode = halt_mode
time.sleep(0.01)
c.command_mode = motion_program_mode

mp_cmds = []

def robot_pose(p,q,conf):
    ret = robot_pose_type()
    ret.tcp_pose[0]["orientation"]["w"] = q[0]
    ret.tcp_pose[0]["orientation"]["x"] = q[1]
    ret.tcp_pose[0]["orientation"]["y"] = q[2]
    ret.tcp_pose[0]["orientation"]["z"] = q[3]
    ret.tcp_pose[0]["position"]["x"] = p[0]*1e-3
    ret.tcp_pose[0]["position"]["y"] = p[1]*1e-3
    ret.tcp_pose[0]["position"]["z"] = p[2]*1e-3

    ret.joint_position_seed=np.zeros((6,))
    ret.joint_position_seed[0] = conf[0]*np.pi/2
    ret.joint_position_seed[3] = conf[1]*np.pi/2
    ret.joint_position_seed[5] = conf[2]*np.pi/2

    return ret

def movej(robot_pose,velocity,blend_radius,fine_point):
    cmd = movej_type()
    cmd.tcp_pose = robot_pose
    cmd.tcp_velocity = velocity
    cmd.blend_radius = blend_radius
    cmd.fine_point = fine_point
    return RR.VarValue(cmd,"experimental.robotics.motion_program.MoveJCommand")

mm = egm_minmax_type()
mm.min = -1e-3
mm.max = 1e-3

egm_config_pt = egm_config_pt_type()
egm_config_pt.corr_frame = geom_util.xyz_rpy_to_pose([0,0,0],[0,0,0])
egm_config_pt.corr_fr_type = egmframetype["EGM_FRAME_BASE"]
egm_config_pt.sensor_frame = geom_util.xyz_rpy_to_pose([0,0,0],[0,0,0])
egm_config_pt.sensor_fr_type = egmframetype["EGM_FRAME_BASE"]
egm_config_pt.x = mm
egm_config_pt.y = mm
egm_config_pt.z = mm
egm_config_pt.rx = mm
egm_config_pt.ry = mm
egm_config_pt.rz = mm
egm_config_pt.max_position_deviation = 1000
egm_config_pt.max_speed_deviation = 1000

setup_cmds = [RR.VarValue(egm_config_pt,"experimental.abb_robot.motion_program.EGMPoseTargetConfigCommand")]

r1 = robot_pose([400,0,600],[0.7071068, 0., 0.7071068, 0.],(0,0,0,1))

mp_cmds.append(movej(r1,5,0.02,True))

run_egm = run_egm_type()
run_egm.cond_time = 10
run_egm.ramp_in_time = 0.05
run_egm.ramp_out_time = 0.05
run_egm.offset = geom_util.xyz_rpy_to_pose([0,0,0],[0,0,0])
mp_cmds.append(RR.VarValue(run_egm,"experimental.abb_robot.motion_program.EGMRunPoseCommand"))

mp = motionprogram_type()

mp.motion_setup_commands = setup_cmds
mp.motion_program_commands = mp_cmds

robot_state_w = c.robot_state.Connect()
robot_state_w.WaitInValueValid()
egm_cmd_w = c.egm_pose_command.Connect()

mp_gen = c.execute_motion_program(mp, False)
res = None

t1 = time.perf_counter()

egm_keep_going = True

pose_target = RRN.ArrayToNamedArray([0.7071068, 0., 0.7071068, 0., 0.4,0,0.6], pose_dtype)

def egm_send_thread():
    time.sleep(1)
    t1 = time.perf_counter()
    seqno = 1
    while egm_keep_going:
        t2 = time.perf_counter()
        pose_target2 = np.copy(pose_target)
        pose_target2[0]["position"]["y"] = (t2-t1)*0.1
        st = robot_state_w.InValue
        pt = egm_pt_type()        
        seqno += 1
        pt.seqno = seqno
        pt.state_seqno = st.seqno        
        pt.cartesian = pose_target2
        egm_cmd_w.OutValue = pt
        time.sleep(0.01)

thread = threading.Thread(target=egm_send_thread)
thread.daemon = True
thread.start()

with suppress(RR.StopIterationException,RR.OperationAbortedException):
    while True:
        res = mp_gen.Next()
        print(f"res.status: {res.action_status}, res.cmd: {res.current_command}, res.queued: {res.queued_command}")

        t2 = time.perf_counter()
        if t2 -t1 > 5:
            mp_gen.Abort()

egm_keep_going = False
        
print("Done!")