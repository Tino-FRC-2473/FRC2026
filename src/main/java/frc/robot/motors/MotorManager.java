package frc.robot.motors;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import frc.robot.Robot;

/**
 * The motor manager that facilitates pooling and logging motor outputs.
 */
public class MotorManager {

	/** A global list containing all logged motors. */
	private static List<LoggedMotor> motorList = new ArrayList<>();

	/**
	 * Updates all motors in the global list and logs information to Ascope.
	 */
	public static void update() {
		for (LoggedMotor motor : motorList) {

			// Update motor simulations
			if (Robot.isSimulation()) {
				motor.updateSimState();
			}

			// Log motor information
			String prefix = MotorConstants.LOGGING_PREFIX;
			Logger.recordOutput(prefix + motor.getIdentifier()
					+ " Rotations", motor.getLoggedPosition());
			Logger.recordOutput(prefix + motor.getIdentifier()
					+ " Velocity", motor.getLoggedVelocity());
			Logger.recordOutput(prefix + motor.getIdentifier()
					+ " Setpoint", motor.getLoggedSetpoint());
			Logger.recordOutput(prefix + motor.getIdentifier()
					+ " Voltage", motor.getLoggedVoltage());
			Logger.recordOutput(prefix + motor.getIdentifier()
					+ " Current", motor.getLoggedCurrent());
		}
	}

	/**
	 * Resets the motor list to an empty list.
	 */
	public static void reset() {
		motorList.clear();
	}

	/**
	 * Adds a motor to the motor list.
	 * @param motor the motor to add
	 */
	public static void addMotor(LoggedMotor motor) {
		motorList.add(motor);
	}
}
