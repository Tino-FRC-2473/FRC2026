// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import java.util.Optional;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import frc.robot.Constants.VisionConstants;
import frc.robot.auto.AutoPaths;
import frc.robot.imported.LimelightHelpers;


// WPILib Imports

// Systems

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.input.AutoInput;
import frc.robot.input.TeleopInput;
import frc.robot.motors.MotorManager;
import frc.robot.systems.Drivetrain;
import frc.robot.systems.Vision;
import frc.robot.systems.FSMSystem;
import frc.robot.systems.IntakeFSMSystem;
import frc.robot.systems.PlaceholderFSMSystem;
import frc.robot.systems.AgitatorFSMSystem;
import frc.robot.systems.ShooterFSMSystem;
//imports for auto chooser
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends LoggedRobot {

	public static final boolean IS_BLUE = true;

	// Robot input
	private AutoInput autoInput;
	private TeleopInput teleopInput;

	// Systems
	private FSMSystem<Drivetrain.DrivetrainState> drivetrainFSMSystem;
	private FSMSystem<IntakeFSMSystem.IntakeFSMState> intakeFSMSystem;
	private FSMSystem<ShooterFSMSystem.ShooterFSMState> shooterFSMSystem;
	private FSMSystem<AgitatorFSMSystem.AgitatorFSMState> agitatorFSMSystem;

	private Vision vision;
	// create sendable chooser
	private final SendableChooser<Command> autoChooser = new SendableChooser<>();
	private Command autonomousCommand;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		System.out.println("robotInit");

		Logger.recordMetadata("FRC 2473", "REBUILT");
		Logger.addDataReceiver(new NT4Publisher());
		Logger.start();

		// // Creates UsbCamera and MjpegServer [1] and connects them
		// CameraServer.startAutomaticCapture(0);
		// // Creates the CvSink and connects it to the UsbCamera
		// CvSink cvSink = CameraServer.getVideo();
		// // Creates the CvSource and MjpegServer [2] and connects them
		// CvSource outputStream = CameraServer.putVideo("Driver Camera",
		// VisionConstants.RESOLUTION_X,
		// VisionConstants.RESOLUTION_Y);

		// Instantiate all systems here
		if (HardwareMap.isDrivetrainEnabled()) {
			Drivetrain drivetrain = new Drivetrain();
			drivetrainFSMSystem = drivetrain;
			vision = new Vision(
					drivetrain::addVisionMeasurement,
					drivetrain.getDrivetrainRotation(),
					VisionConstants.LIMELIGHT_NAME,
					drivetrain::getAngularVelocity,
					drivetrain::getLinearVelocityFromEncoders);
		} else {
			drivetrainFSMSystem = new PlaceholderFSMSystem<>();
			vision = null;
		}
		Optional<IntakeFSMSystem> intake;
		if (HardwareMap.isIntakeEnabled()) {
			intake = Optional.of(new IntakeFSMSystem());
			intakeFSMSystem = intake.get();
		} else {
			intakeFSMSystem = new PlaceholderFSMSystem<>();
			intake = Optional.empty();
		}

		Optional<ShooterFSMSystem> shooter;
		if (HardwareMap.isShooterEnabled()) {
			if (HardwareMap.isDrivetrainEnabled()) {
				shooter = Optional.of(
						new ShooterFSMSystem((Drivetrain) drivetrainFSMSystem));
			} else {
				shooter = Optional.of(
						new ShooterFSMSystem());
			}

			shooterFSMSystem = shooter.get();
		} else {
			shooterFSMSystem = new PlaceholderFSMSystem<>();
			shooter = Optional.empty();
		}

		Optional<AgitatorFSMSystem> agitator;
		if (HardwareMap.isAgitatorEnabled()) {
			agitator = Optional.of(
					new AgitatorFSMSystem(
							intake.isPresent() ? intake.get()::getIsIntakeOuttaking : null,
							shooter.isPresent() ? shooter.get()::getIsFeeding : null));
			agitatorFSMSystem = agitator.get();
		} else {
			agitatorFSMSystem = new PlaceholderFSMSystem<>();
			agitator = Optional.empty();
		}

		autoInput = new AutoInput();
		teleopInput = new TeleopInput();

		if (drivetrainFSMSystem instanceof Drivetrain
				&& shooterFSMSystem instanceof ShooterFSMSystem
				&& intakeFSMSystem instanceof IntakeFSMSystem) {

			Drivetrain drive = (Drivetrain) drivetrainFSMSystem;
			ShooterFSMSystem shooterAuto = (ShooterFSMSystem) shooterFSMSystem;
			IntakeFSMSystem intakeAuto = (IntakeFSMSystem) intakeFSMSystem;
			AutoPaths.loadCommands(
					autoChooser,
					autoInput,
					drive,
					shooterAuto,
					intakeAuto);
		}
		SmartDashboard.putData("Auto Chooser", autoChooser);
		PathfindingCommand.warmupCommand().schedule();
		FollowPathCommand.warmupCommand().schedule();

	}

	@Override
	public void autonomousInit() {
		System.out.println("-------- Autonomous Init --------");
		autoInput.reset();
		drivetrainFSMSystem.reset();
		intakeFSMSystem.reset();
		shooterFSMSystem.reset();

		// get the selected auto
		autonomousCommand = autoChooser.getSelected();
		// scudule auto command
		if (autonomousCommand != null) {
			CommandScheduler.getInstance().enable();
			CommandScheduler.getInstance().schedule(autonomousCommand);
		}
	}

	@Override
	public void autonomousPeriodic() {
		drivetrainFSMSystem.update(autoInput);
		intakeFSMSystem.update(autoInput);
		shooterFSMSystem.update(autoInput);
		agitatorFSMSystem.update(autoInput);
		autoInput.update();
		CommandScheduler.getInstance().run();

		// logs motor values
		MotorManager.update();
	}

	@Override
	public void teleopInit() {
		// set the limelight imu mode for optimal performance in teleop
		LimelightHelpers.SetIMUMode(VisionConstants.LIMELIGHT_NAME,
				VisionConstants.TELEOP_IMU_MODE);

		System.out.println("-------- Teleop Init --------");
		teleopInput.reset();
		CommandScheduler.getInstance().cancelAll();
		CommandScheduler.getInstance().disable();
		drivetrainFSMSystem.reset();
		intakeFSMSystem.reset();
		shooterFSMSystem.reset();
		agitatorFSMSystem.reset();
	}

	@Override
	public void teleopPeriodic() {
		drivetrainFSMSystem.update(teleopInput);
		intakeFSMSystem.update(teleopInput);
		shooterFSMSystem.update(teleopInput);
		agitatorFSMSystem.update(teleopInput);
		teleopInput.update();

		// logs motor values
		MotorManager.update();
	}

	@Override
	public void disabledInit() {
		System.out.println("-------- Disabled Init --------");
		// Pre-match/Disabled: Use mode 1 to continuously seed the internal IMU with
		// external gyro
		LimelightHelpers.SetIMUMode(VisionConstants.LIMELIGHT_NAME, VisionConstants.AUTO_IMU_MODE);
	}

	@Override
	public void disabledPeriodic() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testInit() {
		System.out.println("-------- Test Init --------");
	}

	@Override
	public void testPeriodic() {

	}

	/* Simulation mode handlers, only used for simulation testing */
	@Override
	public void simulationInit() {
		System.out.println("-------- Simulation Init --------");
	}

	@Override
	public void simulationPeriodic() {

	}

	// Do not use robotPeriodic. Use mode specific periodic methods instead.
	@Override
	public void robotPeriodic() {
		if (vision != null) {
			vision.periodic();
		}
		// if (HubTracker.timeRemainingInCurrentShift().isPresent()) {
		// 	Logger.recordOutput("Hub/TimeUntilNextShift",
		// 			HubTracker.timeRemainingInCurrentShift().get());
		// } else {
		// 	Logger.recordOutput("Hub/TimeUntilNextShift",
		// 			-1);

		// }
		// Logger.recordOutput("Hub/IsActive", HubTracker.isActive());
	}
}
