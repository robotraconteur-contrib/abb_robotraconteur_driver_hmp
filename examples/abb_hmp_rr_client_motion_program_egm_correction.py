# Example Robot Raconteur client using EGM path correction with motion program commands
#
# This example demonstrates using EGM path correction with a motion program. The ABB hmp driver provides the
# "EGMPathCorrectionConfigCommand", "EGMMoveL", and "EGMMoveC" commands. Path correction is enabled by passing 
# the "EGMPathCorrectionConfigCommand" as a setup command, then using the "EGMMoveL" and/or "EGMMoveC" commands
# in the motion program. The Robot Raconteur wire "egm_path_correction_command" is used to send the path
# correction command to the EGM. The path correction is in ABB "path coordinates", not the tool, world, or base
# frame. See the `CorrConn` RAPID command documentation for a detailed overview of ABB path coordinates.

# This example is quite verbose because it builds up each of the command structures to send to the driver. A more
# practical use cases would put most of these functions into a utility module.

import math
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
egm_config_corr_type = RRN.GetStructureType("experimental.abb_robot.motion_program.EGMPathCorrectionConfigCommand",c)
egm_minmax_type = RRN.GetStructureType("experimental.abb_robot.motion_program.egm_minmax",c)
motionprogram_type = RRN.GetStructureType("experimental.robotics.motion_program.MotionProgram",c)
egm_corr_type = RRN.GetStructureType("experimental.abb_robot.motion_program.EGMPathCorrection",c)
pose_dtype = RRN.GetNamedArrayDType("com.robotraconteur.geometry.Pose", c)
egmmovel_type = RRN.GetStructureType("experimental.abb_robot.motion_program.EGMMoveLCommand",c)
egmmovec_type = RRN.GetStructureType("experimental.abb_robot.motion_program.EGMMoveCCommand",c)

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

    cfx_j_v = [(45.,45.,45.),(45.,45.,-45),(80.,-135.,45.),(80.,-135.,-45.),
            (-80.,30.,45.),(-80.,30.,-45.),(-35.,-135.,45.),(-35.,-135.,-45.)]
    cfx_j = np.deg2rad(cfx_j_v[int(conf[3])])
    ret.joint_position_seed[1] = cfx_j[0]
    ret.joint_position_seed[2] = cfx_j[1]
    ret.joint_position_seed[4] = cfx_j[2]

    return ret

def movej(robot_pose,velocity,blend_radius,fine_point):
    cmd = movej_type()
    cmd.tcp_pose = robot_pose
    cmd.tcp_velocity = velocity
    cmd.blend_radius = blend_radius
    cmd.fine_point = fine_point
    return RR.VarValue(cmd,"experimental.robotics.motion_program.MoveJCommand")

def egmmovel(robot_pose,velocity,blend_radius,fine_point):
    cmd = egmmovel_type()
    cmd.tcp_pose = robot_pose
    cmd.tcp_velocity = velocity
    cmd.blend_radius = blend_radius
    cmd.fine_point = fine_point
    return RR.VarValue(cmd,"experimental.abb_robot.motion_program.EGMMoveLCommand")

def egmmovec(robot_via_pose,robot_pose,velocity,blend_radius,fine_point):
    cmd = egmmovec_type()
    cmd.tcp_pose = robot_pose
    cmd.tcp_via_pose = robot_via_pose
    cmd.tcp_velocity = velocity
    cmd.blend_radius = blend_radius
    cmd.fine_point = fine_point
    return RR.VarValue(cmd,"experimental.abb_robot.motion_program.EGMMoveCCommand")

mm = egm_minmax_type()
mm.min = -1e-3
mm.max = 1e-3

egm_config_corr = egm_config_corr_type()
egm_config_corr.sensor_frame = geom_util.xyz_rpy_to_pose([0,0,0],[0,0,0])

setup_cmds = [RR.VarValue(egm_config_corr,"experimental.abb_robot.motion_program.EGMPathCorrectionConfigCommand")]

r1 = robot_pose([400,10,600],[0.7071068, 0., 0.7071068, 0.],(0,0,0,1))
r2 = robot_pose([400,200,600],[0.7071068, 0., 0.7071068, 0.],(0,0,0,1))
r3 = robot_pose([400,400,800],[0.7071068, 0., 0.7071068, 0.],(0,0,0,1))

mp_cmds.append(movej(r1,0.5,0.02,True))
mp_cmds.append(egmmovel(r2,0.05,0.001,False))
mp_cmds.append(egmmovel(r1,0.05,0.001,False))
mp_cmds.append(egmmovec(r2,r3,0.05,0.001,False))

mp = motionprogram_type()

mp.motion_setup_commands = setup_cmds
mp.motion_program_commands = mp_cmds

robot_state_w = c.robot_state.Connect()
robot_state_w.WaitInValueValid()
egm_cmd_w = c.egm_path_correction_command.Connect()

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
        corr = egm_corr_type()        
        seqno += 1
        corr.seqno = seqno
        corr.state_seqno = st.seqno        
        corr.pos = geom_util.xyz_to_point([0,math.sin((t2-t1)*10)*10,0])
        egm_cmd_w.OutValue = corr
        time.sleep(0.01)

thread = threading.Thread(target=egm_send_thread)
thread.daemon = True
thread.start()

with suppress(RR.StopIterationException,RR.OperationAbortedException):
    while True:
        res = mp_gen.Next()
        print(f"res.status: {res.action_status}, res.cmd: {res.current_command}, res.queued: {res.queued_command}")

egm_keep_going = False
        
print("Done!")
