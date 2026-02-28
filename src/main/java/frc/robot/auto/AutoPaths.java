package frc.robot.auto;

import static edu.wpi.first.units.Units.Seconds;

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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.input.AutoInput;
import frc.robot.input.InputTypes.ButtonInput;
import frc.robot.systems.ClimberFSMSystem;
import frc.robot.systems.IntakeFSMSystem;
import frc.robot.systems.Drivetrain;
import frc.robot.systems.ShooterFSMSystem;
import frc.robot.systems.ShooterFSMSystem.ShooterFSMState;

public class AutoPaths {

	enum DrivePaths {
		BlueHubNZCimb2,
		BlueLTNZClimb,
		RedHubNZClimb,
		RedLTNZClimb,
		BlueHubNZClimb,
		BlueRTNZClimb,
		RedHubNZClimb2,
		RedRTNZClimb,
		BlueS1_D,
		BlueS2_D,
		BlueS3_D,
		BlueHUB_T,
		BlueD_T,
		BlueD_HUB,
		RedS1_D,
		RedS2_D,
		RedS3_D,
		RedHUB_T,
		RedD_T,
		RedD_HUB,
		BlueD_INTAKE,
		RedD_INTAKE;

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
	 * Returns an auto command that goes from blue S1 to depot,
	 * intakes, and then shoots, then climbs.
	 * @param input the auto input
	 * @param drivetrain the drivetrain
	 * @param shooter the shooter
	 * @param climber the climber
	 * @param intake the intake
	 * @return the auto as a command
	 */
	public static Command getBlueS1DepoShootClimb(
		AutoInput input,
		Drivetrain drivetrain,
		ShooterFSMSystem shooter,
		ClimberFSMSystem climber,
		IntakeFSMSystem intake

	) {
		return new CommandComposer()
			//fold out intake
			.doNext(DrivePaths.BlueS1_D.get())
			//start intake
			.doNext(DrivePaths.BlueD_INTAKE.get())
			//stop intake
			//fold in
			.doNext(DrivePaths.BlueD_HUB.get())
			//shoot for 2-3 sceonds
			//stop shooting
			//extend clibmer
			.doNext(DrivePaths.BlueHUB_T.get())
			//retract climber
			.close();
	}

	/**
	 * Returns a test auto that drives with the the BlueHubNZCimb2 trajectory,
	 * and then shoots in the direction its facing for 10 seconds.
	 * @param input the auto input
	 * @param drivetrain the drivetrain
	 * @param shooter the shooter
	 * @return the auto as a command
	 */
	public static Command getTestAuto(
		AutoInput input,
		Drivetrain drivetrain,
		ShooterFSMSystem shooter
	) {
		return new CommandComposer()
			.doNext(DrivePaths.BlueHubNZCimb2.get())
			.doNext(startShootingCommand(input, shooter))
			.with(new WaitCommand(Seconds.of(2)))
			.doNext(stopShootingCommand(input, shooter))
			.close();
	}

	private static Command startShootingCommand(AutoInput input, ShooterFSMSystem shooter) {
		return new CommandComposer()
			.with(input.pressButtonCommand(ButtonInput.SHOOTER_PREP_TOGGLE))
			.with(input.setButtonCommand(ButtonInput.REV_FEEDER, true))
			.with(shooter.watchForStatesCommand(ShooterFSMState.FEED_STATE))
			.close();
	}

	private static Command stopShootingCommand(AutoInput input, ShooterFSMSystem shooter) {
		return new CommandComposer()
			.with(input.setButtonCommand(ButtonInput.REV_FEEDER, false))
			.with(input.setButtonCommand(ButtonInput.SHOOTER_PREP_TOGGLE, false))
			.with(shooter.watchForStatesCommand(ShooterFSMState.IDLE_STATE))
			.close();
	}

	private static final class CommandComposer {
		private List<Command> commandSequence = new ArrayList<>();
		private List<Command> currentCommandGroup = new ArrayList<>();

		private CommandComposer doNext(Command command) {
			completeCommand();
			currentCommandGroup.add(command);
			return this;
		}

		private CommandComposer with(Command command) {
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
