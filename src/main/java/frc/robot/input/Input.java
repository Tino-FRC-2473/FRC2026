package frc.robot.input;

public sealed interface Input permits TeleopInput, AutoInput {

	/* ======================== Public methods ======================== */
	// Getter methods for fetch input values should be defined here.
	// Method names should be descriptive of the behavior, so the
	// control mapping is hidden from other classes.

	/* ------------------------ Left Joystick ------------------------ */

	/**
	 * Get X axis of Left Joystick.
	 * @return Axis value
	 */
	double getLeftJoystickX();

	/**
	 * Get Y axis of Left Joystick.
	 * @return Axis value
	 */

	double getLeftJoystickY();
	/**
	 * Get the value of the shooter button.
	 * @return True if button is pressed
	 */
	boolean isShooterButtonPressed();

	/**
	 * Get the value of the intake button.
	 * @return True if button is pressed
	 */
	boolean isIntakeButtonPressed();

	/* ------------------------ Right Joystick ------------------------ */

	/**
	 * Get X axis of Right Joystick.
	 * @return Axis value
	 */
	double getRightJoystickX();

	/**
	 * Get Y axis of Right Joystick.
	 * @return Axis value
	 */
	double getRightJoystickY();

}
