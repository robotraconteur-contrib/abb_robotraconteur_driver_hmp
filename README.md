<p align="center"><img src="https://robotraconteurpublicfiles.s3.amazonaws.com/RRheader2.jpg"></p>

# ABB Robot Raconteur Driver Hybrid Motion Program (HMP)

The ABB Robot Raconteur Driver Hybrid Motion Program (HMP) is a sophisticated driver capable of controlling an ABB
IRC5 based robot using either streaming commands using Externally Guided Motion (EGM), or through "Motion Programs".
Motion programs are sequences of commands that are uploaded to the ABB robot controller, and executed using the
motion computer. These motion programs usually consist of MoveAbsJ, MoveJ, MoveL, MoveC, and various other motion
primitive commands. The HMP driver is capable of automatically switching between these modes.

The HMP driver implements three major robot interface types:

* `com.robotraconteur.robotics.robot.Robot` - The "standard robot" type for Robot Raconteur. This robot type uses
  streaming position commands through EGM, and supports 3 command modes: `jog`, `trajectory_command`, and
  `position_command`. In these modes, the HMP driver is fully compatibly with clients expecting a standard 
  Robot Raconteur `Robot` driver.
* `experimental.robotics.motion_program.MotionProgramRobot` - `MotionProgramRobot` is a candidate new standard type
  for robots that support motion programs. Many robot controllers do not have a streaming command interface, and are
  limited to executing scripted motion programs. The ``MotionProgramRobot`` and enclosing service type are designed
  for these motion program only drivers. In the future, this service type will be included in the standard service
  definitions.
* `experimental.abb_robot.ABBRobot` - The `ABBRobot` type implements both `Robot` and `MotionProgramRobot`, allowing
  clients to use either interface. In the future a `HybridCommandRobot` standard type will also be considered. The
  `ABBRobot` object type also provides proprietary support such as wires to send commands to EGM.

The `experimental.abb_robot.motion_program` service definition contains ABB proprietary motion primitive commands.

The HMP driver is based on several Python packages: `RobotRaconteur`, `RobotRaconteurCompanion`, `abb-robot-client`,
`abb-motion-program-exec`, `robotraconteur-abstract-robot`.

## Installation

The driver can be installed using pip:

```
pip install abb-robotraconteur-driver-hmp
```

A robot info file is also necessary. These can be found in the `config/` directory of the GitHub repository. These
files contain metadata and kinematic information about the robot. The robot info file must match the model of the robot.
Open a discussion if your robot does not have a robot info file.

The HMP driver uses the RAPID software from the `abb-motion-program-exec` package
(https://github.com/rpiRobotics/abb_motion_program_exec). The repository contains detailed instructions on
how to install the robot software. **EGM is required, and the EGM configuration must be used when the robot 
is configured.** Currently the HMP driver requires version 0.6.0 of the RAPID software, but this may increase
depending on the version of `abb-motion-program-exec` installed by pip.

Once the `abb-motion-program-exec` robot software is installed, there are additional steps required:

* Edit the `T_ROB1`/`motion_program_exec` RAPID module. This can be done in Robot Studio on the RAPID page, selecting
  `RAPID`/`T_ROB1`/`motion_program_exec` under the controller on the left pane tree. Modify the 
  `MOTION_PROGRAM_DRIVER_MODE` line to show `CONST num MOTION_PROGRAM_DRIVER_MODE:=1`. This will enable the
  operation for drivers.
* Set the "Run Mode" to "Continuous"

Restart the controller. At this point, the controller should be ready.

## Running the driver

First, start the controller. In auto mode, this is done by pressing "Reset PP to Main", and then pressing "Play".

Next, start the driver:

```
python -m abb_robotraconteur_driver_hmp --robot-info=config/abb_1200_5_90_robot_default_config.yml
```

Replace the filename specified for `--robot-info=` with the path to the robot info file for your robot.

## Using the driver

The driver creates a Robot Raconteur service that clients connect to command the robot. See the `examples/` folder
for examples using the driver. Also See https://github.com/robotraconteur/robotraconteur for more information on 
Robot Raconteur and how to communicate with the service.

## Examples

Several example clients are provided in the `examples/` directory demonstrating how to operate the robot in 
different command modes:

* `examples/abb_hmp_rr_client_jog.py` - Client demonstrating using `jog` command mode. Robot must be in manual mode.
* `examples/abb_hmp_rr_client_egm_trajectory_control.py` - Client demonstrating `trajectory` command mode, which
  uses EGM to command the robot to follow a prepared dense joint waypoint trajectory.
* `examples/abb_hmp_rr_client_position_control.py` - Client demonstrating `position_command` command mode, which passes
  streaming joint position commands from the client to the robot controller using EGM.
* `examples/abb_hmp_rr_motion_program.py` - Client demonstrating `motion_program` command mode, sending a sequence of
  motion primitives to the robot controller.
* `examples/abb_hmp_rr_motion_program_freeform.py` - Client demonstrating `motion_program` command mode, sending a 
  sequence of motion primitives to the robot controller using `FreeformCommand` structures.
* `examples/abb_hmp_rr_client_motion_program_preempt.py` - Client demonstration `motion_program` command mode preempting
  the executing of a motion program sequence with updated commands.
* `examples/abb_hmp_rr_client_motion_program_egm_joint.py` Client demonstrating `motion_program` command mode
  with ABB proprietary EGM joint control commands.
* `examples/abb_hmp_rr_client_motion_program_egm_pose.py` Client demonstrating `motion_program` command mode
  with ABB proprietary EGM pose control commands.
* `examples/abb_hmp_rr_client_motion_program_egm_correction.py` Client demonstrating `motion_program` command mode
  with ABB proprietary EGM path correction commands.

## License

Apache 2.0

## Acknowledgment

This work was supported in part by Subaward No. ARM-TEC-21-02-F19 from the Advanced Robotics for Manufacturing ("ARM") Institute under Agreement Number W911NF-17-3-0004 sponsored by the Office of the Secretary of Defense. ARM Project Management was provided by Christopher Adams. The views and conclusions contained in this document are those of the authors and should not be interpreted as representing the official policies, either expressed or implied, of either ARM or the Office of the Secretary of Defense of the U.S. Government. The U.S. Government is authorized to reproduce and distribute reprints for Government purposes, notwithstanding any copyright notation herein.

This work was supported in part by the New York State Empire State Development Division of Science, Technology and Innovation (NYSTAR) under contract C160142. 

![](docs/figures/arm_logo.jpg) ![](docs/figures/nys_logo.jpg)