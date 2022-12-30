# Example Robot Raconteur client using freeform motion program commands
#
# This example is identical to "abb_hmp_rr_client_motion_program.py", except it uses "FreeformCommand" for the
# motion program commands instead of typed structures for each command. FreeformCommand is intended to allow
# for commands that do not have a dedicated structure type, or for situations where it is undesirable to
# deal with a specific type for each command. The available commands are available from the driver under
# the "motion_program_robot_info" property, returning the "MotionProgramRobotInfo" structure. This structure
# contains a list of the commands supported by the driver.

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

time.sleep(0.1)


robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", c)
abb_robot_const = RRN.GetConstants("experimental.abb_robot", c)

halt_mode = robot_const["RobotCommandMode"]["halt"]
motion_program_mode = abb_robot_const["ABBRobotCommandMode"]["motion_program"]


c.command_mode = halt_mode
time.sleep(0.1)
c.command_mode = motion_program_mode

robot_pose_type = RRN.GetStructureType("experimental.robotics.motion_program.RobotPose",c)
freeform_command_type = RRN.GetStructureType("experimental.robotics.motion_program.FreeformCommand",c)
motionprogram_type = RRN.GetStructureType("experimental.robotics.motion_program.MotionProgram",c)
toolinfo_type = RRN.GetStructureType("com.robotraconteur.robotics.tool.ToolInfo",c)
payloadinfo_type = RRN.GetStructureType("com.robotraconteur.robotics.payload.PayloadInfo",c)
transform_dt = RRN.GetNamedArrayDType("com.robotraconteur.geometry.Transform",c)
spatialinertia_dt = RRN.GetNamedArrayDType("com.robotraconteur.geometry.SpatialInertia",c)

toolinfo = toolinfo_type()
toolinfo.tcp = RRN.ArrayToNamedArray([1,0,0,0,0,0,0.001],transform_dt)
toolinfo.inertia = RRN.ArrayToNamedArray([0.1,0,0,0.01,.001,0,0,.001,0,.001],spatialinertia_dt)

settool = freeform_command_type()
settool.command_name = "SetTool"
settool.command_args = {
    "tool_info": RR.VarValue(toolinfo,"com.robotraconteur.robotics.tool.ToolInfo")
}

ii2 = np.zeros((1,),dtype=spatialinertia_dt)
ii2[0]["m"] = 0.10013
ii2[0]["com"] = (1.0023,2.0033,0.00003)
ii2[0]["ixx"] = 0.20043
ii2[0]["ixy"] = 0.050053
ii2[0]["ixz"] = 0.00063
ii2[0]["iyy"] = 0.050073
ii2[0]["iyz"] = 0.00083
ii2[0]["izz"] = 0.2500093

payloadinfo = payloadinfo_type()
payloadinfo.inertia = ii2

setpayload = freeform_command_type()
setpayload.command_name = "SetPayload"
setpayload.command_args = {"payload_info" : RR.VarValue(payloadinfo, "com.robotraconteur.robotics.payload.PayloadInfo"),
"payload_pose": RR.VarValue(geom_util.xyz_rpy_to_pose([0.0073,0.00773, 0.007773], np.deg2rad([1,2,3])),
    "com.robotraconteur.geometry.Pose")
}


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
    cmd = freeform_command_type()
    cmd.command_name = "MoveAbsJ"
    cmd.command_args = {
        "joint_position": RR.VarValue(j, "double[]"),
        "tcp_velocity": RR.VarValue(velocity, "double[]"),
        "blend_radius": RR.VarValue(blend_radius, "double"),
        "fine_point": RR.VarValue(fine_point, "bool")
    }
    return RR.VarValue(cmd,"experimental.robotics.motion_program.FreeformCommand")

def movel(robot_pose,velocity,blend_radius,fine_point):
    cmd = freeform_command_type()
    cmd.command_name = "MoveL"
    cmd.command_args = {
        "tcp_pose": RR.VarValue(robot_pose, "experimental.robotics.motion_program.RobotPose"),
        "tcp_velocity": RR.VarValue(velocity, "double[]"),
        "blend_radius": RR.VarValue(blend_radius, "double"),
        "fine_point": RR.VarValue(fine_point, "bool")
    }
    return RR.VarValue(cmd,"experimental.robotics.motion_program.FreeformCommand")

def movej(robot_pose,velocity,blend_radius,fine_point):
    cmd = freeform_command_type()
    cmd.command_name = "MoveJ"
    cmd.command_args = {
        "tcp_pose": RR.VarValue(robot_pose, "experimental.robotics.motion_program.RobotPose"),
        "tcp_velocity": RR.VarValue(velocity, "double[]"),
        "blend_radius": RR.VarValue(blend_radius, "double"),
        "fine_point": RR.VarValue(fine_point, "bool")
    }
    return RR.VarValue(cmd,"experimental.robotics.motion_program.FreeformCommand")

def movec(robot_via_pose,robot_pose,velocity,blend_radius,fine_point):
    cmd = freeform_command_type()
    cmd.command_name = "MoveC"
    cmd.command_args = {
        "tcp_pose": RR.VarValue(robot_pose, "experimental.robotics.motion_program.RobotPose"),
        "tcp_via_pose": RR.VarValue(robot_via_pose, "experimental.robotics.motion_program.RobotPose"),
        "tcp_velocity": RR.VarValue(velocity, "double"),
        "blend_radius": RR.VarValue(blend_radius, "double"),
        "fine_point": RR.VarValue(fine_point, "bool")
    }
    return RR.VarValue(cmd,"experimental.robotics.motion_program.FreeformCommand")

setup_cmds = [RR.VarValue(settool,"experimental.robotics.motion_program.FreeformCommand"),
    RR.VarValue(setpayload,"experimental.robotics.motion_program.FreeformCommand")]
mp_cmds.append(moveabsj(j1,0.5,0.2,False))
mp_cmds.append(moveabsj(j2,1,0.1,True))
mp_cmds.append(moveabsj(j3,2,0.3,False))
mp_cmds.append(moveabsj(j1,0.5,0.2,True))

mp_cmds.append(movel(r1,0.5,0.02,False))
mp_cmds.append(movel(r2,0.5,0.2,True))

mp_cmds.append(movej(r1,0.5,0.02,False))
mp_cmds.append(movej(r3,0.5,0.2,True))

wt = freeform_command_type()
wt.command_name = "WaitTime"
wt.command_args = {
    "time": RR.VarValue(1.0,"double")
}
mp_cmds.append(RR.VarValue(wt,"experimental.robotics.motion_program.FreeformCommand"))

cirmode = freeform_command_type()
cirmode.command_name = "CirPathMode"
cirmode.command_args = {
    "switch": RR.VarValue("CirPointOri","string")
}
mp_cmds.append(RR.VarValue(cirmode,"experimental.robotics.motion_program.FreeformCommand"))

mp_cmds.append(movec(r4,r5,0.5,0.2,True))

mp = motionprogram_type()

mp.motion_setup_commands = setup_cmds
mp.motion_program_commands = mp_cmds

mp_gen = c.execute_motion_program(mp, False)
res = None

with suppress(RR.StopIterationException):
    while True:
        res = mp_gen.Next()
        print(f"res.status: {res.action_status}, res.cmd: {res.current_command}, res.queued: {res.queued_command}")
        
print("Done!")

