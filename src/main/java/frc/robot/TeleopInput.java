package frc.robot;

// WPILib Imports
import edu.wpi.first.wpilibj.PS4Controller;

/**
 * Common class for providing driver inputs during Teleop.
 *
 * This class is the sole owner of WPILib input objects and is responsible for
 * polling input values. Systems may query TeleopInput via its getter methods
 * for inputs by value, but may not access the internal input objects.
 */
public class TeleopInput {
	/* ======================== Constants ======================== */
	private static final int DRIVE_CONTROLLER_PORT = 0;
	private static final int MECH_CONTROLLER_PORT = 1;

	/* ======================== Private variables ======================== */
	// Input objects
	private PS4Controller driveController;
	private PS4Controller mechController;

	/* ======================== Constructor ======================== */
	/**
	 * Create a TeleopInput and register input devices. Note that while inputs
	 * are registered at robot initialization, valid values will not be provided
	 * by WPILib until teleop mode.
	 */
	public TeleopInput() {
		driveController = new PS4Controller(DRIVE_CONTROLLER_PORT);
		mechController = new PS4Controller(MECH_CONTROLLER_PORT);
	}

	/* ======================== Public methods ======================== */
	// Getter methods for fetch input values should be defined here.
	// Method names should be descriptive of the behavior, so the
	// control mapping is hidden from other classes.

	/* ------------------------ Left Joystick ------------------------ */
	/**
	 * Get X axis of Left Joystick.
	 * @return Axis value
	 */
	public double getLeftJoystickX() {
		return mechController.getLeftX();
	}
	/**
	 * Get Y axis of Left Joystick.
	 * @return Axis value
	 */
	public double getLeftJoystickY() {
		return mechController.getLeftY();
	}
	/**
	 * Get the value of the shooter button.
	 * @return True if button is pressed
	 */
	public boolean isShooterButtonPressed() {
		return mechController.getRawButton(1);
	}
	/**
	 * Get the value of the intake button.
	 * @return True if button is pressed
	 */
	public boolean isIntakeButtonPressed() {
		return mechController.getRawButton(2);
	}

	/**
	 * Get boolean determining if the down button is pressed.
	 * @return true or false depending on state of button
	 */
	public boolean isDownButtonPressed() {
		return true;
	}

	/* ------------------------ Right Joystick ------------------------ */
	/**
	 * Get X axis of Right Joystick.
	 * @return Axis value
	 */
	public double getRightJoystickX() {
		return mechController.getRightX();
	}
	/**
	 * Get Y axis of Right Joystick.
	 * @return Axis value
	 */
	public double getRightJoystickY() {
		return mechController.getRightY();
	}
	/**
	 * Get the value of the climber button.
	 * @return True if button is pressed
	 */
	public boolean isManualOverideButtonPressed() {
		return mechController.getTriangleButtonPressed();
	}
	/**
	 * Get the value of the emergency abort button.
	 * @return True if button is pressed
	 */
	public boolean isNextButtonPressed() {
		return mechController.getSquareButtonPressed();
	}
	/**
	 * Get the value of the emergency abort button.
	 * @return True if button is pressed
	 */
	public boolean isEmergencyAbortPressed() {
		return mechController.getR1ButtonPressed();
	}

	/**
	 * Get the climber manual control input.
	 * @return Control input value
	 */
	public double getClimberManualControl() {
		return getRightJoystickY();
	}

	/* ======================== Private methods ======================== */

}
