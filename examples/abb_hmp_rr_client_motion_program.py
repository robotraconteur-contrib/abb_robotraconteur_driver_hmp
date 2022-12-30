# Example Robot Raconteur client using motion program commands
#
# The "motion_program" command mode works differently than other command modes. Instead of using EGM,
# the motion program mode sends a list of commands to the controller, and uses the vendor motion computer to
# execute the commands. This has the advantage of faster motions with better accuracy, but the input is limited
# to the motion commands the robot controller understands, and the kinematic constraints of these commands. For 
# ABB robots, these commands consist of AbsMoveJ, MoveJ, MoveL, and MoveC. Each command has an associated velocity and
# blending radius. See the ABB command reference guides for more information on these commands.

# This example is quite verbose because it builds up each of the command structures to send to the driver. A more
# practical use cases would put most of these functions into a utility module.

from RobotRaconteur.Client import *
import time
import numpy as np
from contextlib import suppress
from RobotRaconteurCompanion.Util.GeometryUtil import GeometryUtil

c = RRN.ConnectService('rr+tcp://localhost:59925?service=robot')

geom_util = GeometryUtil(client_obj = c)

robot_info = c.robot_info
print(robot_info)

c.reset_errors()

time.sleep(1)


robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", c)
abb_robot_const = RRN.GetConstants("experimental.abb_robot", c)
abb_robot_mp_const = RRN.GetConstants("experimental.abb_robot.motion_program", c)

halt_mode = robot_const["RobotCommandMode"]["halt"]
motion_program_mode = abb_robot_const["ABBRobotCommandMode"]["motion_program"]
cir_path_mode_switch = abb_robot_mp_const["CirPathModeSwitch"]

c.disable_motion_program_mode()
c.enable_motion_program_mode()

robot_pose_type = RRN.GetStructureType("experimental.robotics.motion_program.RobotPose",c)
moveabsj_type = RRN.GetStructureType("experimental.robotics.motion_program.MoveAbsJCommand",c)
movej_type = RRN.GetStructureType("experimental.robotics.motion_program.MoveJCommand",c)
movel_type = RRN.GetStructureType("experimental.robotics.motion_program.MoveLCommand",c)
movec_type = RRN.GetStructureType("experimental.robotics.motion_program.MoveCCommand",c)
settool_type = RRN.GetStructureType("experimental.robotics.motion_program.SetToolCommand",c)
setpayload_type = RRN.GetStructureType("experimental.robotics.motion_program.SetPayloadCommand",c)
waittime_type = RRN.GetStructureType("experimental.robotics.motion_program.WaitTimeCommand",c)
cirpathmode_type = RRN.GetStructureType("experimental.abb_robot.motion_program.CirPathModeCommand",c)
motionprogram_type = RRN.GetStructureType("experimental.robotics.motion_program.MotionProgram",c)
toolinfo_type = RRN.GetStructureType("com.robotraconteur.robotics.tool.ToolInfo",c)
payloadinfo_type = RRN.GetStructureType("com.robotraconteur.robotics.payload.PayloadInfo",c)
transform_dt = RRN.GetNamedArrayDType("com.robotraconteur.geometry.Transform",c)
spatialinertia_dt = RRN.GetNamedArrayDType("com.robotraconteur.geometry.SpatialInertia",c)

toolinfo = toolinfo_type()
toolinfo.tcp = RRN.ArrayToNamedArray([0.9987059,-0.0011418,0.026152,0.0436044,0.0035,0.0015,0.005],transform_dt)

ii = np.zeros((1,),dtype=spatialinertia_dt)
ii[0]["m"] = 0.1
ii[0]["com"] = (1,2,0)
ii[0]["ixx"] = 0.2
ii[0]["ixy"] = 0.05
ii[0]["ixz"] = 0
ii[0]["iyy"] = 0.05
ii[0]["iyz"] = 0
ii[0]["izz"] = 0.25
toolinfo.inertia = ii
#toolinfo.inertia = RRN.ArrayToNamedArray([0.1,0,0,0.01,.001,0,0,.001,0,.001],)

settool = settool_type()
settool.tool_info = toolinfo

ii2 = np.zeros((1,),dtype=spatialinertia_dt)
ii2[0]["m"] = 0.1001
ii2[0]["com"] = (1.002,2.003,0)
ii2[0]["ixx"] = 0.2004
ii2[0]["ixy"] = 0.05005
ii2[0]["ixz"] = 0.0006
ii2[0]["iyy"] = 0.05007
ii2[0]["iyz"] = 0.0008
ii2[0]["izz"] = 0.250009

payloadinfo = payloadinfo_type()
payloadinfo.inertia = ii2

setpayload = setpayload_type()
setpayload.payload_info = payloadinfo
setpayload.payload_pose = geom_util.xyz_rpy_to_pose([0.007,0.0077, 0.00777], np.deg2rad([1,2,3]))


j1 = np.deg2rad(np.array([10,20,30,40,50,60],dtype=np.float64))
j2 = np.deg2rad(np.array([-10,15,-10,10,95,-95],dtype=np.float64))
j3 = np.deg2rad(np.array([15,-5,-25,83,-84,85],dtype=np.float64))


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


r1 = robot_pose([350., -100., 600.], [ 0.0868241, -0.0868241, 0.9924039, 0.0075961 ], (-1,0,-1,0))
r2 = robot_pose([370., 120., 620. ], [ 0.0868241, 0.0868241, 0.9924039, -0.0075961], (0,-1,0,0))

r3 = robot_pose([400., -200., 500.], [0.7071068, 0., 0.7071068, 0.], ( -1.,  -3., 2.,  0.))
r4 = robot_pose([400., 0., 580.], [0.7071068, 0., 0.7071068, 0.], ( 0.,  -3., 2.,  0.))
r5 = robot_pose([400., 200., 500.], [0.7071068, 0., 0.7071068, 0.], ( 0.,  -2., 1.,  0.))

mp_cmds = []

def moveabsj(j,velocity,blend_radius,fine_point):
    cmd = moveabsj_type()
    cmd.joint_position = j
    cmd.tcp_velocity = velocity
    cmd.blend_radius = blend_radius
    cmd.fine_point = fine_point
    return RR.VarValue(cmd,"experimental.robotics.motion_program.MoveAbsJCommand")

def movel(robot_pose,velocity,blend_radius,fine_point):
    cmd = movel_type()
    cmd.tcp_pose = robot_pose
    cmd.tcp_velocity = velocity
    cmd.blend_radius = blend_radius
    cmd.fine_point = fine_point
    return RR.VarValue(cmd,"experimental.robotics.motion_program.MoveLCommand")

def movej(robot_pose,velocity,blend_radius,fine_point):
    cmd = movej_type()
    cmd.tcp_pose = robot_pose
    cmd.tcp_velocity = velocity
    cmd.blend_radius = blend_radius
    cmd.fine_point = fine_point
    return RR.VarValue(cmd,"experimental.robotics.motion_program.MoveJCommand")

def movec(robot_via_pose,robot_pose,velocity,blend_radius,fine_point):
    cmd = movec_type()
    cmd.tcp_pose = robot_pose
    cmd.tcp_via_pose = robot_via_pose
    cmd.tcp_velocity = velocity
    cmd.blend_radius = blend_radius
    cmd.fine_point = fine_point
    return RR.VarValue(cmd,"experimental.robotics.motion_program.MoveCCommand")

setup_cmds = [RR.VarValue(settool,"experimental.robotics.motion_program.SetToolCommand"),
     RR.VarValue(setpayload,"experimental.robotics.motion_program.SetPayloadCommand")]
mp_cmds.append(moveabsj(j1,0.5,0.2,False))
mp_cmds.append(moveabsj(j2,1,0.1,True))
mp_cmds.append(moveabsj(j3,2,0.3,False))
mp_cmds.append(moveabsj(j1,0.5,0.2,True))

mp_cmds.append(movel(r1,0.5,0.02,False))
mp_cmds.append(movel(r2,0.5,0.2,True))

mp_cmds.append(movej(r1,0.5,0.02,False))
mp_cmds.append(movej(r3,0.5,0.2,True))

wt = waittime_type()
wt.time = 1.0
mp_cmds.append(RR.VarValue(wt,"experimental.robotics.motion_program.WaitTimeCommand"))

cirpath = cirpathmode_type()
cirpath.switch = cir_path_mode_switch["CirPointOri"]
mp_cmds.append(RR.VarValue(cirpath,"experimental.abb_robot.motion_program.CirPathModeCommand"))

mp_cmds.append(movec(r4,r5,0.5,0.2,True))

mp = motionprogram_type()

mp.motion_setup_commands = setup_cmds
mp.motion_program_commands = mp_cmds

mp_gen = c.execute_motion_program_record(mp, False)
res = None

with suppress(RR.StopIterationException):
    while True:
        res = mp_gen.Next()
        print(f"res.status: {res.action_status}, res.cmd: {res.current_command}, res.queued: {res.queued_command}")
        
print(f"recording handle {res.recording_handle}")

recording_gen = c.read_recording(res.recording_handle)
recording = recording_gen.NextAll()[0]

c.clear_recordings()

import matplotlib.pyplot as plt
fig, ax1 = plt.subplots()
lns1 = ax1.plot(recording.time, recording.joints)
ax1.set_xlabel("Time (s)")
ax1.set_ylabel("Joint angle (deg)")
ax2 = ax1.twinx()
lns2 = ax2.plot(recording.time, recording.command_number, '-k')
ax2.set_ylabel("Command number")
ax2.set_yticks(range(-1,int(max(recording.command_number))+1))
ax1.legend(lns1 + lns2, recording.column_headers[2:] + ["cmdnum"])
ax1.set_title("Joint motion")
plt.show()

print("Done!")

