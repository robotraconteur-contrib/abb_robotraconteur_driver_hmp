from contextlib import suppress
from RobotRaconteur.RobotRaconteurPythonUtil import NamedArrayToArray
import numpy as np
import abb_motion_program_exec as abb_exec
import RobotRaconteur as RR

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

cmd_get_arg_sentinel = object()
cmd_arg_no_default = object()

def cmd_get_arg(cmd, arg_name, default_value = cmd_arg_no_default):
    if isinstance(cmd, RR.VarValue):
        cmd = cmd.data

    val = getattr(cmd, arg_name, cmd_get_arg_sentinel)
    if val is cmd_get_arg_sentinel:
        freeform_args = getattr(cmd, "command_args", cmd_get_arg_sentinel)
        assert freeform_args is not cmd_get_arg_sentinel, f"Invalid command type, missing argument {arg_name}"

        val = freeform_args.get(arg_name, cmd_get_arg_sentinel)
        if val is cmd_get_arg_sentinel and default_value is not cmd_arg_no_default:
            return default_value
        assert val is not cmd_get_arg_sentinel, f"Invalid command type, missing argument {arg_name}"

    if isinstance(val, RR.VarValue):
        val = val.data

    return val

class MoveAbsJCommandConv:
    rr_types = ["experimental.robotics.motion_program.MoveAbsJCommand"]
    freeform_names = ["MoveAbsJ", "MoveAbsJCommand", "experimental.robotics.motion_program.MoveAbsJCommand"]

    def apply_rr_command(self, cmd, mp):
        zd = rr_zone_to_abb(cmd_get_arg(cmd,"fine_point"),cmd_get_arg(cmd,"blend_radius"))
        sd = rr_speed_to_abb(cmd_get_arg(cmd, "tcp_velocity"))
        jt = rr_joints_to_abb(cmd_get_arg(cmd,"joint_position"), cmd_get_arg(cmd,"joint_units", []))
        mp.MoveAbsJ(jt, sd, zd)

class MoveJCommandConv:
    rr_types = ["experimental.robotics.motion_program.MoveJCommand"]
    freeform_names = ["MoveJ","MoveJCommand","experimental.robotics.motion_program.MoveJCommand"]

    def apply_rr_command(self, cmd, mp):
        zd = rr_zone_to_abb(cmd_get_arg(cmd,"fine_point"),cmd_get_arg(cmd,"blend_radius"))
        sd = rr_speed_to_abb(cmd_get_arg(cmd,"tcp_velocity"))
        rt = rr_robot_pose_to_abb(cmd_get_arg(cmd,"tcp_pose"))
        mp.MoveJ(rt, sd, zd)

class MoveLCommandConv:
    rr_types = ["experimental.robotics.motion_program.MoveLCommand"]
    freeform_names = ["MoveL","MoveLCommand","experimental.robotics.motion_program.MoveLCommand"]

    def apply_rr_command(self, cmd, mp):
        zd = rr_zone_to_abb(cmd_get_arg(cmd,"fine_point"),cmd_get_arg(cmd,"blend_radius"))
        sd = rr_speed_to_abb(cmd_get_arg(cmd,"tcp_velocity"))
        rt = rr_robot_pose_to_abb(cmd_get_arg(cmd,"tcp_pose"))
        mp.MoveL(rt, sd, zd)

class MoveCCommandConv:
    rr_types = ["experimental.robotics.motion_program.MoveCCommand"]
    freeform_names = ["MoveC","MoveCCommand","experimental.robotics.motion_program.MoveCCommand"]

    def apply_rr_command(self, cmd, mp):
        zd = rr_zone_to_abb(cmd_get_arg(cmd,"fine_point"),cmd_get_arg(cmd,"blend_radius"))
        sd = rr_speed_to_abb(cmd_get_arg(cmd,"tcp_velocity"))
        rt = rr_robot_pose_to_abb(cmd_get_arg(cmd,"tcp_pose"))
        rt2 = rr_robot_pose_to_abb(cmd_get_arg(cmd,"tcp_via_pose"))
        mp.MoveC(rt2, rt,  sd, zd)

class WaitTimeCommandConv:
    rr_types = ["experimental.robotics.motion_program.WaitTimeCommand"]
    freeform_names = ["WaitTime", "SetToolCommand", "experimental.robotics.motion_program.WaitTimeCommand"]

    def apply_rr_command(self, cmd, mp):
        mp.WaitTime(cmd_get_arg(cmd, "time"))

class SetToolCommandConv:
    rr_types = ["experimental.robotics.motion_program.SetToolCommand"]
    freeform_names = ["SetTool", "SetToolCommand", "experimental.robotics.motion_program.SetToolCommand"]

    def apply_rr_command(self, cmd, mp):
        assert False, "Unsupported in motion command section"

    def add_setup_args(self, cmd, setup_args):
        abb_tool = rr_tool_to_abb(cmd_get_arg(cmd,"tool_info"))
        setup_args["tool"] = abb_tool

_command_convs = dict()
_freeform_command_convs = dict()

_conv_types = [
    MoveAbsJCommandConv,
    MoveJCommandConv,
    MoveLCommandConv,
    MoveCCommandConv,
    WaitTimeCommandConv,
    SetToolCommandConv
]

def _init_convs():
    for c in _conv_types:
        c_inst = c()
        for x in c_inst.rr_types:
            _command_convs[x] = c_inst
        for y in c_inst.freeform_names:
            _freeform_command_convs[y] = c_inst

_init_convs()

class OptionalCommandException(Exception):
    def __init__(self, message):
        super().__init__(message=message)

def get_command_conv(cmd):
    if cmd.datatype == "experimental.robotics.motion_program.FreeformCommand":
        conv = _freeform_command_convs.get(cmd.data.command_name, None)
        if conv is None:
            if cmd.data.optional:
                raise OptionalCommandException(f"Optional command {cmd.data.command_name}")
            else:
                assert False, f"Unknown command {cmd.data.command_name}"
        return conv
    else:
        conv = _command_convs.get(cmd.datatype, None)
        assert conv is not None, f"Unknown command {cmd.datatype}"
        return conv

def apply_rr_motion_command_to_mp(cmd, mp):
    conv = get_command_conv(cmd)
    conv.apply_rr_command(cmd, mp)

def add_rr_motion_setup_args(cmd, setup_args):
    conv = get_command_conv(cmd)
    conv.add_setup_args(cmd, setup_args)

def rr_motion_program_to_abb(rr_mp):
    setup_args = dict()
    for setup_cmd in rr_mp.motion_setup_commands:
        #with suppress(OptionalCommandException):
            add_rr_motion_setup_args(setup_cmd, setup_args)
    mp = abb_exec.MotionProgram(**setup_args)
    for cmd in rr_mp.motion_program_commands:
        #with suppress(OptionalCommandException):
            apply_rr_motion_command_to_mp(cmd, mp)
        
    return mp