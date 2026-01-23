// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

// WPILib Imports
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
// Systems
import frc.robot.systems.ExampleFSMSystem;
import frc.robot.systems.FSMSystem;
import frc.robot.systems.PlaceholderFSMSystem;
import frc.robot.commands.Autos;
import frc.robot.input.AutoInput;
import frc.robot.input.Input;
import frc.robot.input.TeleopInput;
import frc.robot.motors.MotorManager;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends TimedRobot {
	private Input input;

	// Systems
	private FSMSystem<?> subSystem1;
	private ExampleFSMSystem subSystem2;
	private ExampleFSMSystem subSystem3;

	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		System.out.println("robotInit");

		// Instantiate all systems here
		subSystem2 = new ExampleFSMSystem();
		subSystem3 = new ExampleFSMSystem();

		// you can swap out FSM systems if neccesary
		// this may be needed if you want different behavior in sim
		// do not instantiate something that would try to use hardware you don't have
		if (HardwareMap.isExampleFSMEnabled()) {
			subSystem1 = new ExampleFSMSystem();
		} else {
			subSystem1 = new PlaceholderFSMSystem();
		}

	}

	@Override
	public void autonomousInit() {
		System.out.println("-------- Autonomous Init --------");
		AutoInput autoInput = new AutoInput();
		input = autoInput;
		CommandScheduler.getInstance().schedule(Autos.constructAuto1(autoInput, subSystem2));

	}

	@Override
	public void autonomousPeriodic() {
		subSystem1.update(input);
		subSystem2.update(input);
		subSystem3.update(input);
		input.update();
		CommandScheduler.getInstance().run();

		// logs motor values
		MotorManager.update();
	}

	@Override
	public void teleopInit() {
		System.out.println("-------- Teleop Init --------");
		input = new TeleopInput();
		CommandScheduler.getInstance().cancelAll();
		subSystem1.reset();
		subSystem2.reset();
		subSystem3.reset();
	}

	@Override
	public void teleopPeriodic() {
		subSystem1.update(input);
		subSystem2.update(input);
		subSystem3.update(input);
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
