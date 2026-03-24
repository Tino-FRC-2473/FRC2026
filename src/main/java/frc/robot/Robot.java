// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import java.util.Optional;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import frc.robot.Constants.VisionConstants;
import frc.robot.auto.AutoPaths;
import frc.robot.imported.LimelightHelpers;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;


// WPILib Imports

// Systems

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.input.AutoInput;
import frc.robot.input.Input;
import frc.robot.input.TeleopInput;
import frc.robot.motors.MotorManager;
import frc.robot.systems.Drivetrain;
import frc.robot.systems.Vision;
import frc.robot.systems.FSMSystem;
import frc.robot.systems.IntakeFSMSystem;
import frc.robot.systems.PlaceholderFSMSystem;
import frc.robot.systems.ClimberFSMSystem;
import frc.robot.systems.ShooterFSMSystem;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends LoggedRobot {

	// Robot input
	private Input input;

	// Systems
	private FSMSystem<Drivetrain.DrivetrainState> drivetrainFSMSystem;
	private FSMSystem<ClimberFSMSystem.ClimberFSMState> climberFSMSystem;
	private FSMSystem<IntakeFSMSystem.IntakeFSMState> intakeFSMSystem;
	private FSMSystem<ShooterFSMSystem.ShooterFSMState> shooterFSMSystem;

	private Vision vision;

	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		System.out.println("robotInit");

		Logger.recordMetadata("FRC 2473", "REBUILT");
		Logger.addDataReceiver(new NT4Publisher());
		Logger.start();

		// Creates UsbCamera and MjpegServer [1] and connects them
		CameraServer.startAutomaticCapture(0);
		// Creates the CvSink and connects it to the UsbCamera
		CvSink cvSink = CameraServer.getVideo();
		// Creates the CvSource and MjpegServer [2] and connects them
		CvSource outputStream = CameraServer.putVideo("Driver Camera",
			VisionConstants.RESOLUTION_X,
			VisionConstants.RESOLUTION_Y);

		// Instantiate all systems here
		if (HardwareMap.isDrivetrainEnabled()) {
			Drivetrain drivetrain = new Drivetrain();
			drivetrainFSMSystem = drivetrain;
			vision = new Vision(
				drivetrain::addVisionMeasurement,
				drivetrain.getDrivetrainRotation(),
				VisionConstants.LIMELIGHT_NAME,
				drivetrain::getAngularVelocity,
				drivetrain::getLinearVelocityFromEncoders
			);
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
			shooter = Optional.of(
				new ShooterFSMSystem(
					(Drivetrain) drivetrainFSMSystem,
					(IntakeFSMSystem) intakeFSMSystem)
				);
			shooterFSMSystem = shooter.get();
		} else {
			shooterFSMSystem = new PlaceholderFSMSystem<>();
			shooter = Optional.empty();
		}

		climberFSMSystem = HardwareMap.isClimberEnabled()
			? new ClimberFSMSystem(intake)
			: new PlaceholderFSMSystem<>();
	}

	@Override
	public void autonomousInit() {
		System.out.println("-------- Autonomous Init --------");

		AutoInput autoInput = new AutoInput();
		input = autoInput;
		input.reset();
		drivetrainFSMSystem.reset();
		climberFSMSystem.reset();
		intakeFSMSystem.reset();
		shooterFSMSystem.reset();
		if (drivetrainFSMSystem instanceof Drivetrain drive
			&& shooterFSMSystem instanceof ShooterFSMSystem shooter
			&& climberFSMSystem instanceof ClimberFSMSystem climber
			&& intakeFSMSystem instanceof IntakeFSMSystem intake) {
			System.out.println("Reach 3");
			CommandScheduler.getInstance().schedule(
				AutoPaths.getShootClimbCommand(
					autoInput, drive, shooter, climber, intake,
					new AutoPaths.GetShootClimbSettings(true, true))
			);
		}
	}

	@Override
	public void autonomousPeriodic() {
		drivetrainFSMSystem.update(input);
		climberFSMSystem.update(input);
		intakeFSMSystem.update(input);
		shooterFSMSystem.update(input);

		input.update();
		CommandScheduler.getInstance().run();

		// logs motor values
		MotorManager.update();
	}

	@Override
	public void teleopInit() {
		//set the limelight imu mode for optimal performance in teleop
		LimelightHelpers.SetIMUMode(VisionConstants.LIMELIGHT_NAME,
			VisionConstants.TELEOP_IMU_MODE);

		System.out.println("-------- Teleop Init --------");
		input = new TeleopInput();
		input.reset();
		CommandScheduler.getInstance().cancelAll();
		drivetrainFSMSystem.reset();
		climberFSMSystem.reset();
		intakeFSMSystem.reset();
		shooterFSMSystem.reset();
	}

	@Override
	public void teleopPeriodic() {
		drivetrainFSMSystem.update(input);
		climberFSMSystem.update(input);
		intakeFSMSystem.update(input);
		shooterFSMSystem.update(input);

		input.update();

		// logs motor values
		MotorManager.update();
	}

	@Override
	public void disabledInit() {
		System.out.println("-------- Disabled Init --------");
	}

	@Override
	public void disabledPeriodic() {
		CommandScheduler.getInstance().cancelAll();
		//Pre-match/Disabled: Use mode 1 to continuously seed the internal IMU with external gyro
		LimelightHelpers.SetIMUMode(VisionConstants.LIMELIGHT_NAME, VisionConstants.AUTO_IMU_MODE);
	}

	@Override
	public void testInit() {
		System.out.println("-------- Test Init --------");
	}

	@Override
	public void testPeriodic() {

	}

	/* Simulation mode handlers, only used for simulation testing  */
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
	}
}
