package frc.robot.auto;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.input.AutoInput;
// import frc.robot.input.InputTypes.ButtonInput;
import frc.robot.systems.Drivetrain;
import frc.robot.systems.FSMSystem;
// import frc.robot.systems.Drivetrain.DrivetrainState;

public class AutoPaths {

	enum DrivePaths {
		BlueHubNZCimb2,
		BlueLTNZClimb,
		RedHubNZClimb,
		RedLTNZClimb,
		BlueHubNZClimb,
		BlueRTNZClimb,
		RedHubNZClimb2,
		RedRTNZClimb;

		private PathPlannerPath path;
		DrivePaths() {
			try {
				path = PathPlannerPath.fromChoreoTrajectory(this.name());
			} catch (FileVersionException | IOException | ParseException e) {
				System.err.printf("Failure to load path: %s", this.name());
			}
		}

		Command get() {
			return (path == null || !AutoBuilder.isConfigured())
				? new InstantCommand()
				: AutoBuilder.followPath(path);
		}

	}

	/**
	 * An auto that is only here temporarily for testing purposes.
	 * @param input the auto input
	 * @param drivetrainSystem a system that might be the drivetrain
	 * @return the auto
	 */
	public static Command getTestAuto(AutoInput input,
		FSMSystem<Drivetrain.DrivetrainState> drivetrainSystem) {
		Drivetrain drivetrain;
		if (Drivetrain.class.isInstance(drivetrainSystem)) {
			drivetrain = (Drivetrain) drivetrainSystem;
		} else {
			return new InstantCommand();
		}
		return new AutoComposer()
			// .doNext(input.pressButtonCommand(ButtonInput.DRIVETRAIN_RESEED))
			// .doNext(drivetrain.watchForStatesCommand(DrivetrainState.TELEOP))
			.doNext(DrivePaths.BlueHubNZCimb2.get())
			.close();
	}

	private static final class AutoComposer {
		private List<Command> commandSequence = new ArrayList<>();
		private List<Command> currentCommandGroup = new ArrayList<>();

		private AutoComposer doNext(Command command) {
			completeCommand();
			currentCommandGroup.add(command);
			return this;
		}

		private AutoComposer with(Command command) {
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
