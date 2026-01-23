package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.input.AutoInput;
import frc.robot.input.InputTypes.BooleanSignal;
import frc.robot.systems.ExampleFSMSystem;

public class Autos {

	/**
	 * Constructs and return a command for an example Auto path that doesn't
	 * do anything specific. This is just an example to show how autos should be defined.
	 * @param input the autonInput, which must be periodically updated in autoPeriodic
	 * @param exampleMechanism an instance of the ExampleFSMSystem
	 * @return the constructed Command
	 */
	public static Command constructAuto1(AutoInput input, ExampleFSMSystem exampleMechanism) {
		return new SequentialCommandGroup(
			input.pulseButtonCommand(BooleanSignal.EXAMPLE_BUTTON),
			exampleMechanism.constructObservedStateCommand(
				ExampleFSMSystem.FSMState.OTHER_STATE
			)
		);
	}

}
