# Example Robot Raconteur client using "trajectory" mode control for the ABB robot
#
# The standard robot "trajectory" mode uses a dense set of joint waypoints (<100ms between waypoints) and uses cubic
# interpolation to move the robot through the waypoints. This command mode uses EGM streaming position rather than
# the vendor controller motion computer, such as used by the "motion_program" command mode. This has the advantage
# of tracking an arbitrary path, but may suffer from poor accuracy compared with motion programs.

from RobotRaconteur.Client import *
import time
import numpy as np
from contextlib import suppress

c = RRN.ConnectService('rr+tcp://localhost:59925?service=robot')

robot_info = c.robot_info
print(robot_info)

print(c.robot_state.PeekInValue()[0].command_mode)

robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", c)

joint_names = [j.joint_identifier.name for j in robot_info.joint_info]

halt_mode = robot_const["RobotCommandMode"]["halt"]
trajectory_mode = robot_const["RobotCommandMode"]["trajectory"]

JointTrajectoryWaypoint = RRN.GetStructureType("com.robotraconteur.robotics.trajectory.JointTrajectoryWaypoint",c)
JointTrajectory = RRN.GetStructureType("com.robotraconteur.robotics.trajectory.JointTrajectory",c)

c.reset_errors()
c.enable()

c.command_mode = halt_mode
time.sleep(0.1)
c.command_mode = trajectory_mode


state_w = c.robot_state.Connect()

state_w.WaitInValueValid()
state1 = state_w.InValue

waypoints = []

j_start = state1.joint_position
j_end = [0, 0, 0, 0, 0, 0]

for i in range(101):
    wp = JointTrajectoryWaypoint()
    wp.joint_position = (j_end - j_start)*(float(i)/101.0) + j_start
    wp.time_from_start = i/20.0
    waypoints.append(wp)

traj = JointTrajectory()
traj.joint_names = joint_names
traj.waypoints = waypoints

c.speed_ratio = 1

traj_gen = c.execute_trajectory(traj)


# Next() will raise a RR.StopIterationException when the motion is complete. Suppress the error and break the loop.
with suppress(RR.StopIterationException):
    while (True):
        t = time.time()

        robot_state = state_w.InValue
        res = traj_gen.Next()
        print(res)

        print(hex(c.robot_state.PeekInValue()[0].robot_state_flags))

waypoints = []

for i in range(1001):
    t = float(i)/100.0
    wp = JointTrajectoryWaypoint()
    cmd = np.deg2rad(15)*np.sin(2*np.pi*(t/10.0))*np.array([1,0,0,0,0.5,-1])
    cmd = cmd + j_end
    wp.joint_position = cmd
    wp.time_from_start = t
    waypoints.append(wp)

traj = JointTrajectory()
traj.joint_names = joint_names
traj.waypoints = waypoints

c.speed_ratio = 0.5

traj_gen = c.execute_trajectory(traj)

c.speed_ratio = 2
traj_gen2 = c.execute_trajectory(traj)

res = traj_gen2.Next()
print(res.action_status)

with suppress(RR.StopIterationException):
    while (True):
        t = time.time()

        robot_state = state_w.InValue        
        res = traj_gen.Next()        
        print(res.action_status)

with suppress(RR.StopIterationException):
    while (True):
        t = time.time()

        robot_state = state_w.InValue        
        res = traj_gen2.Next()
        print(res.action_status)
        