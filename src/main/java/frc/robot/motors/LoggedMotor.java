package frc.robot.motors;

/**
 * An interface for any motor wrapper implementation for logging purposes.
 */
public interface LoggedMotor {

	/**
	 * Updates the motors every tick.
	 */
	default void updateSimState() { }

	/**
	 * Initializes the motor by adding it to the global motor list in MotorManager.
	 */
	default void init() {
		MotorManager.addMotor(this);
	}

	/**
	 * Gets the unique identifier of a motor for logging purposes.
	 * This value is intended to be the CAN ID.
	 * @return the unique identifier for the motor
	 */
	String getIdentifier();

	/**
	 * Gets the position of the motor in rotations.
	 * @return the position of the motor in rotations
	 */
	double getLoggedPosition();

	/**
	 * Gets the angular velocity of the motor in rotations per minute.
	 * @return the angular velocity of the motor in rotations per minute
	 */
	double getLoggedVelocity();

	/**
	 * Gets the setpoint of the motor in range [-1, 1].
	 * @return the setpoint in range [-1, 1]
	 */
	double getLoggedSetpoint();

	/**
	 * Gets the voltage of the motor in volts.
	 * @return the voltage of the motor
	 */
	double getLoggedVoltage();

	/**
	 * Gets the current of the motor in amperes.
	 * @return the current of the motor
	 */
	double getLoggedCurrent();

}
