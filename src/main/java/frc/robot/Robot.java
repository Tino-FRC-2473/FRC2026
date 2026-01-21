// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;




// Systems
import frc.robot.systems.ExampleFSMSystem;
import frc.robot.systems.FSMSystem;
import frc.robot.systems.PlaceholderFSMSystem;
import frc.robot.motors.MotorManager;
import frc.robot.systems.AutoHandlerSystem;
import frc.robot.systems.AutoHandlerSystem.AutoPath;
import frc.robot.systems.ClimberFSMSystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends LoggedRobot {
	private TeleopInput input;

	// Systems
	private FSMSystem<?> subSystem1;
	private ExampleFSMSystem subSystem2;
	private ExampleFSMSystem subSystem3;
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

		// Instantiate all systems here
		subSystem2 = new ExampleFSMSystem();
		subSystem3 = new ExampleFSMSystem();
		climberFSMSystem = new ClimberFSMSystem();

		// you can swap out FSM systems if neccesary
		// this may be needed if you want different behavior in sim
		// do not instantiate something that would try to use hardware you don't have
		if (HardwareMap.isExampleFSMEnabled()) {
			subSystem1 = new ExampleFSMSystem();
		} else {
			subSystem1 = new PlaceholderFSMSystem();
		}

		Logger.recordMetadata("sdfjdsj", "ljsdlfkjsdlfj");
		Logger.addDataReceiver(new NT4Publisher());
		Logger.start();
	}

	@Override
	public void autonomousInit() {
		System.out.println("-------- Autonomous Init --------");
		autoHandler.reset(AutoPath.PATH1);
	}

	@Override
	public void autonomousPeriodic() {
		autoHandler.update();

		// logs motor values
		MotorManager.update();
	}

	@Override
	public void teleopInit() {
		System.out.println("-------- Teleop Init --------");
		subSystem1.reset();
		subSystem2.reset();
		subSystem3.reset();
		climberFSMSystem.reset();
	}

	@Override
	public void teleopPeriodic() {
		subSystem1.update(input);
		subSystem2.update(input);
		subSystem3.update(input);
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
