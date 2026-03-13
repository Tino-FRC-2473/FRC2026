package frc.robot.auto;

import java.io.IOException;
import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.numbers.N10;
// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform2d;
// import frc.robot.Constants.DrivetrainConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.input.AutoInput;
import frc.robot.input.InputTypes.AxialInput;
import frc.robot.input.InputTypes.ButtonInput;
import frc.robot.systems.ClimberFSMSystem;
import frc.robot.systems.IntakeFSMSystem;
import frc.robot.systems.Drivetrain;
import frc.robot.systems.ShooterFSMSystem;
import frc.robot.systems.IntakeFSMSystem.IntakeFSMState;
import frc.robot.systems.ShooterFSMSystem.ShooterFSMState;
import frc.robot.systems.ClimberFSMSystem.ClimberFSMState;
import frc.robot.Constants.AutoPathsConstants;

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
		RedD_INTAKE(BlueD_INTAKE),

		NZ_BlueS3,
		NZ_RedS3(NZ_BlueS3),

		BlueS3_HUB,
		RedS3_HUB(BlueS3_HUB),

		NZ_BlueS1,
		NZ_RedS1(NZ_BlueS1),

		BlueS3_NZ,
		RedS3_NZ(BlueS3_NZ),

		HUB_BlueS1,
		HUB_RedS1(HUB_BlueS1),

		BlueNZ_INTAKE,
		RedNZ_INTAKE(BlueNZ_INTAKE),

		HUB_BlueS2,
		HUB_RedS2(HUB_BlueS2),

		BlueS1_HUB,
		RedS1_HUB(BlueS1_HUB),

		HUB_BlueS3,
		HUB_RedS3(HUB_BlueS3),

		NZ_BlueS2,
		NZ_RedS2(NZ_BlueS2),

		BlueS1_NZ,
		RedS1_NZ(BlueS1_NZ),

		BlueS2_HUB,
		RedS2_HUB(BlueS2_HUB),

		BlueS2_NZ,
		RedS2_NZ(BlueS2_NZ);

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

	public enum Start {
		S1(
				DrivePaths.BlueS1_D,
				DrivePaths.BlueS1_NZ,
				DrivePaths.NZ_BlueS1,
				DrivePaths.BlueS1_HUB),
		S2(
				DrivePaths.BlueS2_D,
				DrivePaths.BlueS2_NZ,
				DrivePaths.NZ_BlueS2,
				DrivePaths.BlueS2_HUB),
		S3(
				DrivePaths.BlueS3_D,
				DrivePaths.BlueS3_NZ,
				DrivePaths.NZ_BlueS3,
				DrivePaths.BlueS3_HUB);

		private DrivePaths depotPath;
		private DrivePaths nzPath;
		private DrivePaths nzPathBack;
		private DrivePaths hubPath;

		Start(
				DrivePaths pathToDepot,
				DrivePaths pathToNZ,
				DrivePaths pathFromNZ,
				DrivePaths pathToHub) {
			// mirror matches what we tested on the bot
			// not sure why/if this works and whether I should mirror the others
			depotPath = pathToDepot.mirror();

			nzPath = pathToNZ;
			nzPathBack = pathFromNZ;
			hubPath = pathToHub;
		}
	}

	public record GetShootClimbSettings(
			boolean shouldShoot, boolean isRed) {
	}

	public record GetShootSettings() {
	}

	/**
	 * Returns an auto command that goes from a start position to depot,
	 * intakes, optionally shoots into the hub, then climbs.
	 *
	 * @param input      the auto input
	 * @param drivetrain the drivetrain
	 * @param shooter    the shooter
	 *                   // * @param climber the climber
	 * @param intake     the intake
	 * @param settings   some settings
	 *                   starting postion, and whether it should shoot during auto
	 * @return the auto as a command
	 */
	public static Command getShootCommand(
			AutoInput input,
			Drivetrain drivetrain,
			ShooterFSMSystem shooter,
			// ClimberFSMSystem climber,
			IntakeFSMSystem intake,
			GetShootSettings settings

	) {

		return Commands
				.sequence(
						Commands.waitSeconds(AutoPathsConstants.SHOOT_WAIT_TIME_SECONDS),
						input.setAxisCommand(AxialInput.DRIVETRAIN_DRIVE_X, 0),
						shootFor(input, shooter, N10.instance.getNum()),
						input.setAxisCommand(AxialInput.DRIVETRAIN_DRIVE_X, 1 / 2));

	}

	/**
	 * Returns an auto command that goes from a start position to depot,
	 * intakes, optionally shoots into the hub, then climbs.
	 *
	 * @param input      the auto input
	 * @param drivetrain the drivetrain
	 * @param shooter    the shooter
	 * @param climber    the climber
	 * @param intake     the intake
	 * @param settings   settings
	 *                   starting postion, and whether it should shoot during auto
	 * @return the auto as a command
	 */
	public static Command getShootClimbCommand(
			AutoInput input,
			Drivetrain drivetrain,
			ShooterFSMSystem shooter,
			ClimberFSMSystem climber,
			IntakeFSMSystem intake,
			GetShootClimbSettings settings

	) {

		boolean isRed = settings.isRed();

		// right now isRed = true means blue and isRed = false means red ;(
		System.out.println("Reach 1");

		return Commands
				.sequence(
						shootFor(input, shooter, AutoPathsConstants.SHOOT_CLIMB_SECONDS),
						input.pressButtonCommand(ButtonInput.CLIMBER_AUTO_UP_1),
						climber.watchForStatesCommand(ClimberFSMState.AUTO_UP_1,
								ClimberFSMState.AUTO_IDLE),
						DrivePaths.BlueHUB_T.get(isRed),
						input.pressButtonCommand(ButtonInput.CLIMBER_AUTO_UP_2),
						DrivePaths.BlueS2_HUB.get(isRed));

	}

	public record NZShootClimbSettings(
			boolean shouldShoot, boolean isRed, Start startingPositon) {
	}

	public record DepotShootClimbSettings(
			boolean shouldShoot, boolean isRed, Start startingPositon) {
	}

	/**
	 * Returns an auto command that goes from a start position to depot,
	 * intakes, optionally shoots into the hub, then climbs.
	 *
	 * @param input      the auto input
	 * @param drivetrain the drivetrain
	 * @param shooter    the shooter
	 * @param climber    the climber
	 * @param intake     the intake
	 * @param settings   the setttings, including Blue/Red,
	 *                   starting postion, and whether it should shoot during auto
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
		// right now isRed = true means blue and isRed = false means red ;(
		boolean isRed = settings.isRed();
		boolean shouldShoot = settings.shouldShoot();

		return Commands
				.sequence(
						Commands.parallel(
								settings.startingPositon().depotPath.get(!isRed),
								startIntakeCommand(input, intake)),
						DrivePaths.BlueD_INTAKE.get(isRed),
						stopIntakeCommand(input, intake),
						Commands.either(
								shootFor(input, shooter, 2),
								Commands.none(),
								() -> shouldShoot),
						Commands.parallel(
								input.pressButtonCommand(ButtonInput.CLIMBER_AUTO_UP_1),
								climber.watchForStatesCommand(
										ClimberFSMState.AUTO_UP_1, ClimberFSMState.AUTO_IDLE)),
						DrivePaths.BlueHUB_T.get(isRed),
						input.pressButtonCommand(ButtonInput.CLIMBER_AUTO_UP_2));
	}

	/**
	 * Returns an auto command that goes from a start position to neutral zone,
	 * intakes, optionally shoots into the hub, then climbs.
	 *
	 * @param input      the auto input
	 * @param drivetrain the drivetrain
	 * @param shooter    the shooter
	 * @param climber    the climber
	 * @param intake     the intake
	 * @param settings   the setttings, including Blue/Red,
	 *                   starting postion, and whether it should shoot during auto
	 * @return the auto as a command
	 */
	public static Command getNZShootClimbCommand(
			AutoInput input,
			Drivetrain drivetrain,
			ShooterFSMSystem shooter,
			ClimberFSMSystem climber,
			IntakeFSMSystem intake,
			NZShootClimbSettings settings

	) {
		// right now isRed = true means blue and isRed = false means red ;(
		boolean isRed = settings.isRed();
		boolean shouldShoot = settings.shouldShoot();

		return Commands
				.sequence(
						settings.startingPositon().nzPath.get(isRed),
						startIntakeCommand(input, intake),
						DrivePaths.BlueNZ_INTAKE.get(shouldShoot),
						stopIntakeCommand(input, intake),
						settings.startingPositon().nzPathBack.get(isRed),
						settings.startingPositon().hubPath.get(isRed),
						Commands.either(
								shootFor(input, shooter, 2),
								Commands.none(),
								() -> shouldShoot),
						Commands.parallel(
								input.pressButtonCommand(ButtonInput.CLIMBER_AUTO_UP_1),
								climber.watchForStatesCommand(
										ClimberFSMState.AUTO_UP_1, ClimberFSMState.AUTO_IDLE)),
						DrivePaths.BlueHUB_T.get(isRed),
						input.pressButtonCommand(ButtonInput.CLIMBER_AUTO_UP_2));
	}

	/**
	 * Returns a test auto that drives with the the BlueHubNZCimb2 trajectory,
	 * and then shoots in the direction its facing for 10 seconds.
	 *
	 * @param input      the auto input
	 * @param drivetrain the drivetrain
	 * @param shooter    the shooter
	 * @return the auto as a command
	 */
	public static Command getTestAuto(
			AutoInput input,
			Drivetrain drivetrain,
			ShooterFSMSystem shooter) {
		return Commands
				.sequence(shootFor(input, shooter, N10.instance.getNum()));
	}

	private static Command startShootingCommand(AutoInput input, ShooterFSMSystem shooter) {
		return Commands
				.sequence(
						input.pressButtonCommand(ButtonInput.PASSER_PREP_TOGGLE),
						input.setButtonCommand(ButtonInput.REV_FEEDER, true),
						shooter.watchForStatesCommand(ShooterFSMState.FEED_STATE));
	}

	private static Command startIntakeCommand(AutoInput input, IntakeFSMSystem intake) {
		return Commands
				.sequence(
						Commands.parallel(
								input.pressButtonCommand(ButtonInput.FOLD_OUT_BUTTON),
								intake.watchForStatesCommand(IntakeFSMState.IDLE_OUT_STATE)),
						Commands.parallel(
								input.setButtonCommand(ButtonInput.INTAKE_BUTTON, true),
								intake.watchForStatesCommand(IntakeFSMState.INTAKE_STATE)));
	}

	private static Command stopIntakeCommand(AutoInput input, IntakeFSMSystem intake) {
		return Commands
				.sequence(
						input.setButtonCommand(ButtonInput.INTAKE_BUTTON, false),
						intake.watchForStatesCommand(IntakeFSMState.IDLE_OUT_STATE),
						input.pressButtonCommand(ButtonInput.PARTIAL_OUT_BUTTON),
						intake.watchForStatesCommand(IntakeFSMState.PARTIAL_OUT_STATE));
	}

	private static Command stopShootingCommand(AutoInput input, ShooterFSMSystem shooter) {
		return Commands
				.sequence(
						input.setButtonCommand(ButtonInput.REV_FEEDER, false),
						Commands.waitSeconds(1 / N10.instance.getNum()),
						input.pressButtonCommand(ButtonInput.IDLE_SHOOTER_TOGGLE),
						shooter.watchForStatesCommand(ShooterFSMState.IDLE_STATE));
	}

	private static Command shootFor(AutoInput input, ShooterFSMSystem shooter, double time) {
		return Commands
				.sequence(
						startShootingCommand(input, shooter),
						Commands.waitSeconds(time),
						stopShootingCommand(input, shooter));
	}

	// on the fly path example
	// private static Command getS1HubCommand() {
	// return AutoBuilder.pathfindToPoseFlipped(
	// AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded)
	// .getTagPose(DrivetrainConstants.TAG_TO_ALIGN_TO)
	// .orElse(null).toPose2d()
	// .transformBy(
	// new Transform2d(
	// DrivetrainConstants.X_TRANFORM_FROM_TAG,
	// DrivetrainConstants.Y_TRANFORM_FROM_TAG,
	// Rotation2d.kCCW_90deg
	// )
	// ),
	// DrivetrainConstants.PATH_CONSTRAINTS
	// );
	// }
}
