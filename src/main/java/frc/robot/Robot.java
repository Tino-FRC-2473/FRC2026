// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;




// Systems
import frc.robot.motors.MotorManager;
import frc.robot.systems.Drivetrain;
import frc.robot.systems.AutoHandlerSystem;
import frc.robot.systems.ClimberFSMSystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends LoggedRobot {
	private TeleopInput input;

	// Systems
	private Drivetrain drivetrain;
	private ClimberFSMSystem climberFSMSystem;

	private AutoHandlerSystem autoHandler;

	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		System.out.println("robotInit");
		input = new TeleopInput();

		Logger.recordMetadata("FRC 2473", "REBUILT");
		Logger.addDataReceiver(new NT4Publisher());
		Logger.start();

		// Instantiate all systems here
		if (HardwareMap.isDrivetrainEnabled()) {
			drivetrain = new Drivetrain();
		}
		climberFSMSystem = new ClimberFSMSystem();
	}

	@Override
	public void autonomousInit() {
		System.out.println("-------- Autonomous Init --------");
	}

	@Override
	public void autonomousPeriodic() {

		// logs motor values
		MotorManager.update();
	}

	@Override
	public void teleopInit() {
		System.out.println("-------- Teleop Init --------");
		drivetrain.reset();
		climberFSMSystem.reset();
	}

	@Override
	public void teleopPeriodic() {
		drivetrain.update(input);
		climberFSMSystem.update(input);

		// logs motor values
		MotorManager.update();
	}

	@Override
	public void disabledInit() {
		System.out.println("-------- Disabled Init --------");
	}

	@Override
	public void disabledPeriodic() {

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
	public void robotPeriodic() { }
}
