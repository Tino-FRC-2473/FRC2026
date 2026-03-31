package frc.robot.auto;


import edu.wpi.first.math.numbers.N10;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.math.numbers.*;
// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform2d;
// import frc.robot.Constants.DrivetrainConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.input.AutoInput;
import frc.robot.input.InputTypes.ButtonInput;
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
		RedD_INTAKE(BlueD_INTAKE),

		NZ_BlueS3,
		NZ_RedS3(NZ_BlueS3),

		BlueS3_HUB,
		RedS3_HUB(BlueS3_HUB),

		NZ_BlueS1,
		NZ_RedS1(NZ_BlueS1),

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

		BlueHUB_D,
		RedHUB_D(BlueHUB_D),

		BlueHUB_O,
		RedHUB_O(BlueHUB_O),

		NZ_BlueS2,
		NZ_RedS2(NZ_BlueS2),

		BlueS1_NZ_1,
		RedS1_NZ_1(BlueS1_NZ_1),

		BlueS1_NZ_2,
		RedS1_NZ_2(BlueS1_NZ_2),

		BlueS3_NZ_1,
		RedS3_NZ_1(BlueS3_NZ_1),

		BlueS3_NZ_2,
		RedS3_NZ_2(BlueS3_NZ_2),

		BlueS2_HUB,
		RedS2_HUB(BlueS2_HUB),

		BlueS2_NZ,
		RedS2_NZ(BlueS2_NZ),

		BlueSHOOTS1_D,
		RedSHOOTS1_D(BlueSHOOTS1_D),

		BlueSHOOTS1_O,
		RedSHOOTS1_O(BlueSHOOTS1_O),

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
				DrivePaths.BlueS1_NZ_1,
				DrivePaths.NZ_BlueS1,
				DrivePaths.BlueS1_HUB),
		S2(
				DrivePaths.BlueS2_D,
				DrivePaths.BlueS2_NZ,
				DrivePaths.NZ_BlueS2,
				DrivePaths.BlueS2_HUB),
		S3(
				DrivePaths.BlueS3_D,
				DrivePaths.BlueS3_NZ_1,
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

	public record S1_NZ_Settings(
			boolean shouldShoot, boolean isRed) {
	}

	

	public static Command S1_NZ_Command(AutoInput input,
			Drivetrain drivetrain,
			ShooterFSMSystem shooter,
			// ClimberFSMSystem climber,
			IntakeFSMSystem intake,
			S1_NZ_Settings settings) {

		boolean isRed = settings.isRed();
		return Commands
				.sequence(
					Commands.parallel(
						DrivePaths.BlueS1_NZ_1.get(isRed),
						startIntakeCommand(input, intake)
					),
					stopIntakeCommand(input, intake),
					DrivePaths.NZ_BlueS1.get(isRed),
					faceHub(input, drivetrain),
					waitFor(N1.instance.getNum()),
					shootFor(input, shooter, N5.instance.getNum()),
					Commands.parallel(
						DrivePaths.BlueS1_NZ_2.get(isRed),
						startIntakeCommand(input, intake)
					),
					stopIntakeCommand(input, intake),
					DrivePaths.NZ_BlueS1.get(isRed),
					faceHub(input, drivetrain),
					waitFor(N1.instance.getNum()),
					shootFor(input, shooter, N5.instance.getNum())
				);

	}

	public record S3_NZ_Settings(
			boolean shouldShoot, boolean isRed) {
	}

	

	public static Command S3_NZ_Command(AutoInput input,
			Drivetrain drivetrain,
			ShooterFSMSystem shooter,
			// ClimberFSMSystem climber,
			IntakeFSMSystem intake,
			S3_NZ_Settings settings) {

		boolean isRed = settings.isRed();
		return Commands
				.sequence(
					Commands.parallel(
						DrivePaths.BlueS3_NZ_1.get(isRed),
						startIntakeCommand(input, intake)
					),
					stopIntakeCommand(input, intake),
					DrivePaths.NZ_BlueS3.get(isRed),
					faceHub(input, drivetrain),
					waitFor(N1.instance.getNum()),
					shootFor(input, shooter, N5.instance.getNum()),
					Commands.parallel(
						DrivePaths.BlueS3_NZ_2.get(isRed),
						startIntakeCommand(input, intake)
					),
					stopIntakeCommand(input, intake),
					DrivePaths.NZ_BlueS3.get(isRed),
					faceHub(input, drivetrain),
					waitFor(N1.instance.getNum()),
					shootFor(input, shooter, N5.instance.getNum())
				);

	}

		public record S2_D_Settings(
			boolean shouldShoot, boolean isRed) {
	}

	

	public static Command S2_D_Command(AutoInput input,
			Drivetrain drivetrain,
			ShooterFSMSystem shooter,
			// ClimberFSMSystem climber,
			IntakeFSMSystem intake,
			S2_D_Settings settings) {

		boolean isRed = settings.isRed();
		return Commands
				.sequence(
					DrivePaths.BlueS2_D.get(isRed),
					Commands.parallel(
						startIntakeCommand(input, intake),
						DrivePaths.BlueD_INTAKE.get(isRed)
					),
					stopIntakeCommand(input, intake),
					faceHub(input, drivetrain),
					waitFor(N1.instance.getNum()),
					shootFor(input, shooter, N5.instance.getNum())
				);

	}



	/**
	 * Returns an auto command that goes from a start position to depot,
	 * intakes, optionally shoots into the hub, then climbs.
	 *
	 * @param input      the auto input
	 * @param drivetrain the drivetrain
	 * @param shooter    the shooter
	 *      //@param climber the climber
	 * @param intake     the intake
	 * @param settings   some settings
	 *                   starting postion, and whether it should shoot during auto
	 * @return the auto as a command
	 */
	private static Command getS1HUBShootCommand(
			AutoInput input,
			Drivetrain drivetrain,
			ShooterFSMSystem shooter,
			IntakeFSMSystem intake
	) {
		System.out.println("Getting S1 HUB Shoot Command");
		return Commands
				.sequence(
					drivetrain.followcommand("BlueS1_HUB"),
					shootFor(input, shooter, N10.instance.getNum())
				);
	}

	/**
	 * Returns a test auto that drives with the the BlueS1HUB trajectory,
	 * and then shoots in the direction its facing for 10 seconds.
	 * @param input
	 * @param drivetrain
	 * @param shooter
	 * @param intake
	 * @return Command
	 */
	private static Command getS2HUBShootCommand(
			AutoInput input,
			Drivetrain drivetrain,
			ShooterFSMSystem shooter,
			IntakeFSMSystem intake
	) {
		return Commands
				.sequence(
					drivetrain.followcommand("BlueS2_HUB"),
					shootFor(input, shooter, N10.instance.getNum())
				);
	}

	/**
	 * Returns a test auto that drives with the the BlueS1HUB trajectory,
	 * and then shoots in the direction its facing for 10 seconds.
	 * @param input
	 * @param drivetrain
	 * @param shooter
	 * @param intake
	 * @return Command
	 */
	private static Command getS3HUBShootCommand(
			AutoInput input,
			Drivetrain drivetrain,
			ShooterFSMSystem shooter,
			IntakeFSMSystem intake
	) {
		return Commands
				.sequence(
					drivetrain.followcommand("BlueS3_HUB"),
					shootFor(input, shooter, N10.instance.getNum())
				);
	}

	private static Command getS1NZHUBShootCommand(
			AutoInput input,
			Drivetrain drivetrain,
			ShooterFSMSystem shooter,
			IntakeFSMSystem intake
	) {
		return Commands
				.sequence(
					drivetrain.followcommand("BlueS1_NZ"),
					startIntakeCommand(input, intake),
					drivetrain.followcommand("BlueNZ_INTAKE"),
					stopIntakeCommand(input, intake),
					drivetrain.followcommand("BlueNZ_HUB"),
					shootFor(input, shooter, N10.instance.getNum())
				);
	}

	private static Command getS2NZHUBShootCommand(
			AutoInput input,
			Drivetrain drivetrain,
			ShooterFSMSystem shooter,
			IntakeFSMSystem intake
	) {
		return Commands
				.sequence(
					drivetrain.followcommand("BlueS2_NZ"),
					startIntakeCommand(input, intake),
					drivetrain.followcommand("BlueNZ_INTAKE"),
					stopIntakeCommand(input, intake),
					drivetrain.followcommand("BlueNZ_HUB"),
					shootFor(input, shooter, N10.instance.getNum())
				);
	}

	private static Command getS3NZHUBShootCommand(
			AutoInput input,
			Drivetrain drivetrain,
			ShooterFSMSystem shooter,
			IntakeFSMSystem intake
	) {
		return Commands
				.sequence(
					drivetrain.followcommand("BlueS3_NZ"),
					startIntakeCommand(input, intake),
					drivetrain.followcommand("BlueNZ_INTAKE"),
					stopIntakeCommand(input, intake),
					drivetrain.followcommand("BlueNZ_HUB"),
					shootFor(input, shooter, N10.instance.getNum())
				);
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

	private static Command waitFor(double time) {
		return Commands.waitSeconds(time);
	}

	private static Command faceHub(AutoInput input, Drivetrain drive) {
		return Commands
			.sequence(
				input.pressButtonCommand(ButtonInput.FACE_HUB)
			);
	}

	private static Command facePass(AutoInput input, Drivetrain drive) {
		return Commands
			.sequence(
				input.pressButtonCommand(ButtonInput.FACE_PASS)
			);
	}

	/**
	 * Returns a test auto that drives with the the BlueHubNZCimb2 trajectory,
	 * and then shoots in the direction its facing for 10 seconds.
	 *
	 * @param chooser    the auto chooser
	 * @param input      the auto input
	 * @param drivetrain the drivetrain
	 * @param shooter    the shooter
	 * @param intake     the intake
	 */

	public static void loadCommands(
			SendableChooser<Command> chooser,
			AutoInput input,
			Drivetrain drivetrain,
			ShooterFSMSystem shooter,
			IntakeFSMSystem intake) {
		System.out.println("IT IS LOADING");
		chooser.setDefaultOption(
				"S1 Shoot",
				getS1HUBShootCommand(input, drivetrain, shooter, intake));
		chooser.addOption(
				"S2 Shoot",
				getS2HUBShootCommand(input, drivetrain, shooter, intake));
		chooser.addOption(
				"S3 Shoot",
				getS3HUBShootCommand(input, drivetrain, shooter, intake));
		chooser.addOption(
				"S1 NZ Shoot",
				getS1NZHUBShootCommand(input, drivetrain, shooter, intake));
		chooser.addOption(
				"S2 NZ Shoot",
				getS2NZHUBShootCommand(input, drivetrain, shooter, intake));
		chooser.addOption(
				"S3 NZ Shoot",
				getS3NZHUBShootCommand(input, drivetrain, shooter, intake));
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
