package frc.robot.auto;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DrivetrainConstants;
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
		RedD_INTAKE(BlueD_INTAKE);

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
				for (DrivePaths other : DrivePaths.values()) {
					if (other.mirror != null && other.mirror == this) {
						mirror = other;
						return mirror;
					}
				}
			}
			return mirror;
		}

		Command get(boolean shouldMirror) {
			return shouldMirror ? mirror().get() : get();
		}

	}

	public record DepotShootClimbSettings(
		boolean shouldShoot, boolean isRed, StartingPositon startingPositon) {
		public enum StartingPositon {
			S1(DrivePaths.BlueS1_D),
			S2(DrivePaths.BlueS2_D),
			S3(DrivePaths.BlueS3_D);

			private DrivePaths depotPath;
			StartingPositon(DrivePaths pathToDepot) {
				depotPath = pathToDepot;
			}

			DrivePaths getDepotPath() {
				return depotPath;
			}
		}
	}

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
		// ShooterFSMSystem shooter,
		// ClimberFSMSystem climber,
		// IntakeFSMSystem intake,
		DepotShootClimbSettings settings

	) {
		// right now isRed = true means blue and isRed = false means red ;(
		boolean isRed = settings.isRed();
		boolean shouldShoot = settings.shouldShoot();
		return new CommandComposer()

			// drive from start to blue depo
			.doNext(settings.startingPositon().getDepotPath().get(isRed))
			//start intake
			.doNext(input.setButtonCommand(ButtonInput.INTAKE_BUTTON, true))
			// .with(intake.watchForStatesCommand(IntakeFSMState.INTAKE_STATE))

			// drive through depo
			.doNext(DrivePaths.RedD_INTAKE.get(isRed))

			//stop intake and fold in
			.doNext(input.setButtonCommand(ButtonInput.INTAKE_BUTTON, false))
			.with(input.pressButtonCommand(ButtonInput.PARTIAL_OUT_BUTTON))

			// if we shouldn't shoot ...
			.keepActiveIf(!shouldShoot)
			// raise climber
			.doNext(input.pressButtonCommand(ButtonInput.CLIMBER_NEXT_STEP))
			// .with(climber.waitForExtendedL1())
			// go to climber
			.doNext(DrivePaths.RedD_T.get(isRed))
			.reactivate()

			// if we should shoot ...
			.keepActiveIf(shouldShoot)
			//drive to hub
			.doNext(DrivePaths.RedD_HUB.get(isRed))
			// shoot for 2-3 seconds
			// .doNext(shootFor(input, shooter, 2))
			// extend climber
			.doNext(input.pressButtonCommand(ButtonInput.CLIMBER_NEXT_STEP))
			// .with(climber.waitForExtendedL1())
			// drive to tower
			.doNext(DrivePaths.RedHUB_T.get(isRed))
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
			.doNext(getS1HubCommand())
			.doNext(shootFor(input, shooter, 10))
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

	private static Command getS1HubCommand() {
		return AutoBuilder.pathfindToPoseFlipped(
				AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded)
				.getTagPose(DrivetrainConstants.TAG_TO_ALIGN_TO)
				.orElse(null).toPose2d()
				.transformBy(
					new Transform2d(
						DrivetrainConstants.X_TRANFORM_FROM_TAG,
						DrivetrainConstants.Y_TRANFORM_FROM_TAG,
						Rotation2d.kCCW_90deg
					)
				),
				DrivetrainConstants.PATH_CONSTRAINTS
			);
	}
}