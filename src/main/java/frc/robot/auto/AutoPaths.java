package frc.robot.auto;


import edu.wpi.first.math.numbers.N10;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.numbers.N6;
import edu.wpi.first.math.numbers.N8;

//import org.ejml.equation.Sequence;

import edu.wpi.first.math.numbers.N1;

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
	/**
	 * The S3_NZ_Command function in Java executes a sequence of commands
	 * 		involving driving paths, intake
	 * control, shooting, and facing a target.
	 * @param input The `input` parameter is likely an object that
	 * 		provides input data or commands to the method.
	 * @param drivetrain The `drivetrain` parameter in the S3_NZ_Command`
	 * 		method represents the drivetrain subsystem of the robot.
	 * @param shooter The `shooter` parameter in the
	 * 		`S3_NZ_Command` method represents a ShooterFSMSystem object
	 * @param intake The `intake` parameter in
	 * 		the `S3_NZ_Command` method represents an `IntakeFSMSystem`object.
	 * @return The method `S3_NZ_Command` is returning a
	 * 		`Command` object that represents a sequence of
	 * commands to be executed.
	 */
	public static Command getS3NZCommand(
			AutoInput input,
			Drivetrain drivetrain,
			ShooterFSMSystem shooter,
			IntakeFSMSystem intake) {
		return Commands
			.sequence(
				Commands.parallel(
					drivetrain.followcommand("BlueS3_NZ_1"),
					startIntakeCommand(input, intake)
				),
				stopIntakeCommand(input, intake),
				drivetrain.followcommand("NZ_BlueS3"),
				faceHub(input, drivetrain),
				waitFor(N1.instance.getNum()),
				shootFor(input, shooter, N5.instance.getNum()),
				Commands.parallel(
					drivetrain.followcommand("BlueS3_NZ_2"),
					startIntakeCommand(input, intake)
				),
				stopIntakeCommand(input, intake),
				drivetrain.followcommand("NZ_BlueS3"),
				faceHub(input, drivetrain),
				waitFor(N1.instance.getNum()),
				shootFor(input, shooter, N5.instance.getNum())
			);

	}

	/**
	 * The function `S2_D_Command` executes a sequence of commands for a
	 * 		specific scenario involving a
	 * drivetrain, shooter, and intake systems in a Java program.
	 * @param input The `input` parameter is an object of type `AutoInput`, which likely contains
	 * information or settings needed for autonomous operation.
	 * @param drivetrain The `drivetrain` parameter in the
	 * 		`S2_D_Command` method represents the drivetrain subsystem of the robot.
	 * @param shooter The `shooter` parameter in the `S2_D_Command`
	 * 		method represents a ShooterFSMSystem object.
	 * @param intake The `intake` parameter in the `S2_D_Command`
	 * 		method seems to be an instance of the `IntakeFSMSystem` class.
	 * @return A Command object is being returned.
	 * 		The Command object is created by sequencing a series of c
	 * 		commands using the Commands class.
	 */
	public static Command getS2DepotCommand(
			AutoInput input,
			Drivetrain drivetrain,
			ShooterFSMSystem shooter,
			IntakeFSMSystem intake) {
		return Commands
				.sequence(
					drivetrain.followcommand("BlueS2_D"),
					Commands.parallel(
						startIntakeCommand(input, intake),
						drivetrain.followcommand("BlueD_INTAKE")
					),
					stopIntakeCommand(input, intake),
					faceHub(input, drivetrain),
					waitFor(N1.instance.getNum()),
					shootFor(input, shooter, N5.instance.getNum())
				);

	}

	/**
	 * The function `getS1ShootNzShootCommand` defines a sequence of commands for shooting and
	 * controlling the drivetrain and intake subsystems in a specific order.
	 * @param input The `input` parameter is likely an object or class that provides input data or
	 * commands to the method.
	 * @param drivetrain The `drivetrain` parameter in the `getS1ShootNzShootCommand`
	 * method represents the drivetrain subsystem of a robot.
	 * robot
	 * @param shooter The `shooter` parameter in the `getS1ShootNzShootCommand` method represents a
	 * ShooterFSMSystem object.
	 * @param intake The `intake` parameter in the `getS1ShootNzShootCommand`
	 * method represents an `IntakeFSMSystem` object.
	 * @return The `getS1ShootNzShootCommand` method is returning a Command
	 * object that represents a sequence of actions to be performed.
	 */
	public static Command getS1ShootNzShootCommand(
		AutoInput input,
		Drivetrain drivetrain,
		ShooterFSMSystem shooter,
		IntakeFSMSystem intake
	) {
		return Commands
			.sequence(
				drivetrain.followcommand("S1_S1SHOOTING"),
				waitFor(N1.instance.getNum()),
				shootFor(input, shooter, N5.instance.getNum()),
				drivetrain.followcommand("S1SHOOTING_S1NZ"),
				Commands.parallel(
					startIntakeCommand(input, intake),
					drivetrain.followcommand("S1NZ_INTAKE")
				),
				stopIntakeCommand(input, intake),
				drivetrain.followcommand("S1NZ_S1SHOOTING"),
				waitFor(N1.instance.getNum()),
				shootFor(input, shooter, N8.instance.getNum())
			);
	}


	/**
	 * The function `getS2ShootDepotShootCommand` executes a sequence of
	 * commands for shooting at the depot in a specific order.
	 * @param input The `input` parameter is an object of type `AutoInput`,
	 * which likely contains input data or settings for the autonomous routine.
	 * @param drivetrain The `drivetrain` parameter in the `getS2ShootDepotShootCommand`
	 * method represents the drivetrain subsystem of the robot.
	 * @param shooter The `shooter` parameter in the `getS2ShootDepotShootCommand`
	 * method represents a ShooterFSMSystem object.
	 * @param intake The `intake` parameter in the `getS2ShootDepotShootCommand`
	 * method represents an `IntakeFSMSystem` object.
	 * @return A Command object is being returned.
	 */
	public static Command getS2ShootDepotShootCommand(
		AutoInput input,
		Drivetrain drivetrain,
		ShooterFSMSystem shooter,
		IntakeFSMSystem intake
	) {
		return Commands
			.sequence(
				drivetrain.followcommand("S2_HUB"),
				waitFor(N1.instance.getNum()),
				shootFor(input, shooter, N5.instance.getNum()),
				drivetrain.followcommand("HUB_D"),
				Commands.parallel(
					startIntakeCommand(input, intake),
					drivetrain.followcommand("D_INTAKE")
				),
				stopIntakeCommand(input, intake),
				drivetrain.followcommand("D_HUB"),
				waitFor(N1.instance.getNum()),
				shootFor(input, shooter, N8.instance.getNum())
			);
	}

	/**
	 * This function returns a sequence of commands for shooting at an outpost in a specific order.
	 * @param input The `input` parameter is likely an object or class that provides input data or
	 * commands to the method.
	 * @param drivetrain The `drivetrain` parameter in the `getS3ShootOutpostShootCommand` method
	 * represents the drivetrain subsystem of a robot.
	 * @param shooter The `shooter` parameter in the `getS3ShootOutpostShootCommand`
	 * method is an instance of the `ShooterFSMSystem` class.
	 * @param intake The `intake` parameter in the `getS3ShootOutpostShootCommand`
	 * method represents an `IntakeFSMSystem` object.
	 * @return A Command object is being returned, which is created by sequencing a series
	 * of commands related to driving, shooting, and waiting for specific durations.
	 */
	public static Command getS3ShootOutpostShootCommand(
		AutoInput input,
		Drivetrain drivetrain,
		ShooterFSMSystem shooter,
		IntakeFSMSystem intake
	) {
		return Commands
			.sequence(
				drivetrain.followcommand("S3_S3SHOOTING"),
				waitFor(N1.instance.getNum()),
				shootFor(input, shooter, N5.instance.getNum()),
				drivetrain.followcommand("S3SHOOTING_O"),
				waitFor(N5.instance.getNum()),
				drivetrain.followcommand("O_S3SHOOTING"),
				waitFor(N1.instance.getNum()),
				shootFor(input, shooter, N6.instance.getNum())
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
						input.pressButtonCommand(ButtonInput.SHOOTER_PREP_TOGGLE),
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
		chooser.addOption(
				"S3 NZ command",
				getS3NZCommand(input, drivetrain, shooter, intake));
		chooser.addOption(
			"S2 Depot command",
			getS2DepotCommand(input, drivetrain, shooter, intake));
		chooser.addOption(
			"S3 shoot Outpost Shoot command",
			getS3ShootOutpostShootCommand(input, drivetrain, shooter, intake));
		chooser.addOption(
			"S2 Shoot Depot Shoot command",
			getS2ShootDepotShootCommand(input, drivetrain, shooter, intake));
		chooser.addOption(
			"S1 Shoot NZ Shoot command",
			getS1ShootNzShootCommand(input, drivetrain, shooter, intake));
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
