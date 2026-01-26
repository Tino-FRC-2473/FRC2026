package frc.robot.auto;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.input.AutoInput;
import frc.robot.input.InputTypes.ButtonInput;
import frc.robot.systems.Drivetrain;
import frc.robot.systems.Drivetrain.DrivetrainState;

public class AutoPaths {

	/**
	 * An auto that is only here temporarily for testing purposes.
	 * @param input the auto input
	 * @param drivetrain the drivetrain
	 * @return the auto
	 */
	public static Command getTestAuto(AutoInput input, Drivetrain drivetrain) {
		return new AutoBuilder()
			.doNext(input.pressButtonCommand(ButtonInput.RESEED_DRIVETRAIN))
			.doNext(drivetrain.watchForStatesCommand(DrivetrainState.TELEOP))
			.close();
	}

	private static final class AutoBuilder {
		private List<Command> commandSequence = new ArrayList<>();
		private List<Command> currentCommandGroup = new ArrayList<>();

		private AutoBuilder doNext(Command command) {
			completeCommand();
			currentCommandGroup.add(command);
			return this;
		}

		private AutoBuilder with(Command command) {
			currentCommandGroup.add(command);
			return this;
		}

		private Command close() {
			completeCommand();
			return new SequentialCommandGroup(commandSequence.toArray(new Command[0]));
		}

		private void completeCommand() {
			if (currentCommandGroup.size() > 1) {
				commandSequence.add(new ParallelCommandGroup(
					currentCommandGroup.toArray(new Command[0])
				));
			} else if (currentCommandGroup.size() == 1) {
				commandSequence.add(currentCommandGroup.get(0));
			}

			currentCommandGroup.clear();
		}
	}

}
