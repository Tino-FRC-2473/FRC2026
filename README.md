# FSMBotTemplate

Finite-state-machine based template project for WPILib based robot code.

To provide a more structured framework for FIRST Robotics Competition robot development, this project defines subsystem behaviors strictly in terms of multiple separate finite state machines updated in a round-robin fashion. This will make scheduling behavior explicitly visible instead of hidden behind the command scheduler, and avoid ambiguous shared state between command and subsystems under the WPILib command based programming model.

## FSMSystem
The primary base class for robot systems. Each robot system is defined in terms of a Mealy-style finite state machine with control over a well-defined set of robot hardware. 

## TeleopInput
Utility class with ownership of teleop input handling. The single global instance of this class mediates access to inputs during the teleoperated mode and abstracts control mappings.
