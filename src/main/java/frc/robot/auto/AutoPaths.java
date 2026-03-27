package frc.robot.auto;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.numbers.N10;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
	private static PathPlannerPath blueS1D;
	private static PathPlannerPath blueS2D;
	private static PathPlannerPath blueS3D;
	private static PathPlannerPath blueDT;
	private static PathPlannerPath blueHUBT;
	private static PathPlannerPath blueDHUB;
	private static PathPlannerPath blueDINTAKE;
	private static PathPlannerPath nzBlueS3;
	private static PathPlannerPath blueS3HUB;
	private static PathPlannerPath nzBlueS1;
	private static PathPlannerPath blueS3NZ;
	private static PathPlannerPath hubBlueS1;
	private static PathPlannerPath blueNZINTAKE;
	private static PathPlannerPath hubBlueS2;
	private static PathPlannerPath blueS1HUB;
	private static PathPlannerPath hubBlueS3;
	private static PathPlannerPath nzBlueS2;
	private static PathPlannerPath blueS1NZ;
	private static PathPlannerPath blueS2HUB;
	private static PathPlannerPath blueS2NZ;

	/**
	 * Loads all the paths from the PathPlanner trajectories.
	 */
	public AutoPaths() {
		try {
			blueS1D = PathPlannerPath.fromChoreoTrajectory("BlueS1_D");
			blueS2D = PathPlannerPath.fromChoreoTrajectory("BlueS2_D");
			blueS3D = PathPlannerPath.fromChoreoTrajectory("BlueS3_D");
			blueDT = PathPlannerPath.fromChoreoTrajectory("BlueD_T");
			blueHUBT = PathPlannerPath.fromChoreoTrajectory("BlueHUB_T");
			blueDHUB = PathPlannerPath.fromChoreoTrajectory("BlueD_HUB");
			blueDINTAKE = PathPlannerPath.fromChoreoTrajectory("BlueD_INTAKE");
			nzBlueS3 = PathPlannerPath.fromChoreoTrajectory("NZ_BlueS3");
			blueS3HUB = PathPlannerPath.fromChoreoTrajectory("BlueS3_HUB");
			nzBlueS1 = PathPlannerPath.fromChoreoTrajectory("NZ_BlueS1");
			blueS3NZ = PathPlannerPath.fromChoreoTrajectory("BlueS3_NZ");
			hubBlueS1 = PathPlannerPath.fromChoreoTrajectory("HUB_BlueS1");
			blueNZINTAKE = PathPlannerPath.fromChoreoTrajectory("BlueNZ_INTAKE");
			hubBlueS2 = PathPlannerPath.fromChoreoTrajectory("HUB_BlueS2");
			blueS1HUB = PathPlannerPath.fromChoreoTrajectory("BlueS1_HUB");
			hubBlueS3 = PathPlannerPath.fromChoreoTrajectory("HUB_BlueS3");
			nzBlueS2 = PathPlannerPath.fromChoreoTrajectory("NZ_BlueS2");
			blueS1NZ = PathPlannerPath.fromChoreoTrajectory("BlueS1_NZ");
			blueS2HUB = PathPlannerPath.fromChoreoTrajectory("BlueS2_HUB");
			blueS2NZ = PathPlannerPath.fromChoreoTrajectory("BlueS2_NZ");
		} catch (FileVersionException | IOException | ParseException e) {
			System.err.println("Failure to load paths");
		}
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
					drivetrain.followcommand("blueS2HUB"),
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
					drivetrain.followcommand("blueS3HUB"),
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
		chooser.setDefaultOption(
				"S1 Shoot",
				getS1HUBShootCommand(input, drivetrain, shooter, intake));
		chooser.addOption(
				"S2 Shoot",
				getS2HUBShootCommand(input, drivetrain, shooter, intake));
		chooser.addOption(
				"S3 Shoot",
				getS3HUBShootCommand(input, drivetrain, shooter, intake));
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
