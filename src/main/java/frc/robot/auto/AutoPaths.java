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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.input.AutoInput;
import frc.robot.input.InputTypes.ButtonInput;
import frc.robot.systems.ClimberFSMSystem;
import frc.robot.systems.IntakeFSMSystem;
import frc.robot.systems.Drivetrain;
import frc.robot.systems.ShooterFSMSystem;
import frc.robot.systems.IntakeFSMSystem.IntakeFSMState;
import frc.robot.systems.ShooterFSMSystem.ShooterFSMState;

public class AutoPaths {

	enum DrivePaths {
		BlueHubNZCimb2,
		RedHubNZClimb2(BlueHubNZCimb2),

		BlueLTNZClimb,
		RedLTNZClimb(BlueLTNZClimb),

		BlueHubNZClimb,
		RedHubNZClimb(BlueHubNZClimb),

		BlueRTNZClimb,
		RedRTNZClimb(BlueRTNZClimb),

		BlueS1_D,
		RedS1_D(BlueS1_D),

		BlueS2_D,
		RedS2_D(BlueS2_D),

		BlueS3_D,
		RedS3_D(BlueS3_D),

		BlueD_T,
		RedD_T(BlueD_T),

		BlueHUB_T,
		RedHUB_T(BlueHUB_T),

		BlueD_HUB,
		RedD_HUB(BlueD_HUB),

		BlueD_INTAKE,
		RedD_INTAKE(BlueD_INTAKE),

		BlueS1_HUB,
		RedS1_HUB(BlueS1_HUB),

		BlueS2_HUB,
		RedS2_HUB(BlueS2_HUB),

		BlueS3_HUB,
		RedS3_HUB(BlueS3_HUB);

		private PathPlannerPath path;
		private DrivePaths mirror;

		DrivePaths(DrivePaths mirroredPath) {
			mirror = mirroredPath;
			try {
				path = PathPlannerPath.fromChoreoTrajectory(this.name());
			} catch (FileVersionException | IOException | ParseException e) {
				System.err.printf("Failure to load path: %s", this.name());
			}
		}

		DrivePaths() {
			this(null);
		}

		Command get() {
			return (path == null || !AutoBuilder.isConfigured())
				? new InstantCommand()
				: AutoBuilder.followPath(path);
		}

		DrivePaths mirror() {
			if (mirror == null) {
				for (DrivePaths p : DrivePaths.values()) {
					if (p.mirror != null && p.mirror == this) {
						mirror = p.mirror;
					}
				}
			}
			return mirror;
		}

		Command get(boolean shouldMirror) {
			return shouldMirror ? mirror().get() : get();
		}

	}

	public enum StartingPositon {
		S1(DrivePaths.BlueS1_D, DrivePaths.BlueS1_HUB),
		S2(DrivePaths.BlueS2_D, DrivePaths.BlueS2_HUB),
		S3(DrivePaths.BlueS3_D, DrivePaths.BlueS3_HUB);

		// ad more ___ paths as needed
		private DrivePaths depotPath;
		private DrivePaths hubPath;

		StartingPositon(DrivePaths pathToDepot, DrivePaths pathToHub) {
			depotPath = pathToDepot;
			hubPath = pathToHub;
		}

		DrivePaths getDepotPath() {
			return depotPath;
		}

		DrivePaths getHubPath() {
			return hubPath;
		}
	}

	public record DepotShootClimbSettings(
		boolean shouldShoot, boolean isRed, StartingPositon startingPositon) { }

	/**
	 * Returns an auto command that goes from a start position to depot,
	 * intakes, optionally shoots into the hub, then climbs.
	 * @param input the auto input
	 * @param drivetrain the drivetrain
	 * @param shooter the shooter
	 * @param climber the climber
	 * @param intake the intake
	 * @param settings the setttings, including Blue/Red,
	 * starting postion, and whether it should shoot during auto
	 * @return the auto as a command
	 */
	public static Command getDepotShootClimb(
		AutoInput input,
		Drivetrain drivetrain,
		ShooterFSMSystem shooter,
		ClimberFSMSystem climber,
		IntakeFSMSystem intake,
		DepotShootClimbSettings settings

	) {
		boolean isRed = settings.isRed();
		boolean shouldShoot = settings.shouldShoot();
		return new CommandComposer()

			// drive from start to blue depo
			.doNext(settings.startingPositon().getDepotPath().get(isRed))
			//start intake
			.doNext(input.setButtonCommand(ButtonInput.INTAKE_BUTTON, true))
			.with(intake.watchForStatesCommand(IntakeFSMState.INTAKE_STATE))

			// drive through depo
			.doNext(DrivePaths.BlueD_INTAKE.get(isRed))

			//stop intake and fold in
			.doNext(input.setButtonCommand(ButtonInput.INTAKE_BUTTON, false))
			.with(input.pressButtonCommand(ButtonInput.PARTIAL_OUT_BUTTON))

			// if we shouldn't shoot ...
			.keepActiveIf(!shouldShoot)
			// raise climber
			.doNext(input.pressButtonCommand(ButtonInput.CLIMBER_NEXT_STEP))
			.with(climber.waitForExtendedL1())
			// go to climber
			.doNext(DrivePaths.BlueD_T.get(isRed))
			.reactivate()

			// if we should shoot ...
			.keepActiveIf(shouldShoot)
			//drive to hub
			.doNext(DrivePaths.BlueD_HUB.get(isRed))
			// shoot for 2-3 seconds
			.doNext(shootFor(input, shooter, 2))
			// extend climber
			.doNext(input.pressButtonCommand(ButtonInput.CLIMBER_NEXT_STEP))
			.with(climber.waitForExtendedL1())
			// drive to tower
			.doNext(DrivePaths.BlueHUB_T.get(isRed))
			.reactivate()

			// retract climber
			.doNext(input.pressButtonCommand(ButtonInput.CLIMBER_NEXT_STEP))

			// finish command
			.close();
	}

	public record ShootClimbSettings(
		boolean shouldShoot, boolean isRed, StartingPositon startingPositon) { }

	/**
	 * Returns an auto command that goes from a start position,
	 * goes to the hub, optionally shoots, then climbs.
	 * @param input the auto input
	 * @param drivetrain the drivetrain
	 * @param shooter the shooter
	 * @param climber the climber
	 * @param settings the setttings, including Blue/Red,
	 * starting postion, and whether it should shoot during auto
	 * @return the auto as a command
	 */
	public static Command getShootClimb(
		AutoInput input,
		Drivetrain drivetrain,
		ShooterFSMSystem shooter,
		ClimberFSMSystem climber,
		ShootClimbSettings settings

	) {
		boolean isRed = settings.isRed();
		boolean shouldShoot = settings.shouldShoot();
		return new CommandComposer()
			//drive from starting position to hub
			.doNext(settings.startingPositon().getHubPath().get(isRed))

			// if we shouldn't shoot ...
			.keepActiveIf(!shouldShoot)
			// raise climber
			.doNext(input.pressButtonCommand(ButtonInput.CLIMBER_NEXT_STEP))
			.with(climber.waitForExtendedL1())
			// go to tower
			.doNext(DrivePaths.BlueHUB_T.get(isRed))
			.reactivate()

			// if we should shoot ...
			.keepActiveIf(shouldShoot)
			// shoot for 2-3 seconds
			.doNext(shootFor(input, shooter, 2))
			// extend climber
			.doNext(input.pressButtonCommand(ButtonInput.CLIMBER_NEXT_STEP))
			.with(climber.waitForExtendedL1())
			// drive to tower
			.doNext(DrivePaths.BlueHUB_T.get(isRed))
			.reactivate()

			// retract climber
			.doNext(input.pressButtonCommand(ButtonInput.CLIMBER_NEXT_STEP))

			// finish command
			.close();
	}

	public record ShootNzSettings(
		boolean shouldShoot, boolean isRed, StartingPositon startingPositon) { }

	/**
	 * Returns an auto command that goes from a start position,
	 * goes to the hub, optionally shoots, then climbs.
	 * @param input the auto input
	 * @param drivetrain the drivetrain
	 * @param shooter the shooter
	 * @param climber the climber
	 * @param settings the setttings, including Blue/Red,
	 * starting postion, and whether it should shoot during auto
	 * @return the auto as a command
	 */
	public static Command getShootNz(
		AutoInput input,
		Drivetrain drivetrain,
		ShooterFSMSystem shooter,
		ClimberFSMSystem climber,
		ShootClimbSettings settings

	) {
		boolean isRed = settings.isRed();
		boolean shouldShoot = settings.shouldShoot();
		return new CommandComposer()
			//drive from starting position to hub
			.doNext(settings.startingPositon().getHubPath().get(isRed))

			// if we shouldn't shoot ...
			.keepActiveIf(!shouldShoot)
			// raise climber
			.doNext(input.pressButtonCommand(ButtonInput.CLIMBER_NEXT_STEP))
			.with(climber.waitForExtendedL1())
			// go to tower
			.doNext(DrivePaths.BlueHUB_T.get(isRed))
			.reactivate()

			// if we should shoot ...
			.keepActiveIf(shouldShoot)
			// shoot for 2-3 seconds
			.doNext(shootFor(input, shooter, 2))
			// extend climber
			.doNext(input.pressButtonCommand(ButtonInput.CLIMBER_NEXT_STEP))
			.with(climber.waitForExtendedL1())
			// drive to tower
			.doNext(DrivePaths.BlueHUB_T.get(isRed))
			.reactivate()

			// retract climber
			.doNext(input.pressButtonCommand(ButtonInput.CLIMBER_NEXT_STEP))

			// finish command
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
			.doNext(shootFor(input, shooter, 2))
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

	private static Command shootFor(AutoInput input, ShooterFSMSystem shooter, double time) {
		return new CommandComposer()
			.with(startShootingCommand(input, shooter))
			.with(new WaitCommand(time))
			.doNext(stopShootingCommand(input, shooter))
			.close();
	}

	private static final class CommandComposer {
		private boolean active = true;
		private List<Command> commandSequence = new ArrayList<>();
		private List<Command> currentCommandGroup = new ArrayList<>();

		private CommandComposer keepActiveIf(boolean condition) {
			active &= condition;
			return this;
		}

		private CommandComposer reactivate() {
			active = true;
			return this;
		}

		private CommandComposer doNext(Command command) {
			completeCommand();
			return with(command);
		}

		private CommandComposer with(Command command) {
			if (active) {
				currentCommandGroup.add(command);
			}
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
