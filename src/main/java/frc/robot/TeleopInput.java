package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
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

	/* ------------------------ Driver Controller ------------------------ */
	/**
	 * Get X axis of the left joystick.
	 * @return Axis value
	 */
	public double getDriverLeftX() {
		return driveController.getLeftX();
	public double getLeftJoystickX() {
		return mechController.getLeftX();
	}
	/**
	 * Get Y axis of the left joystick.
	 * @return Axis value
	 */
	public double getDriverLeftY() {
		return driveController.getLeftY();
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
	 * Get X axis of the right joystick.
	 * @return Axis value
	 */
	public double getDriverRightX() {
		return driveController.getRightX();
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
	/**
	 * Get Y axis of the right joystick.
	 * @return Axis value
	 */
	public double getDriverRightY() {
		return driveController.getRightY();
	public double getRightJoystickX() {
		return mechController.getRightX();
	}

	/**
	 * Get share button state for drive controller.
	 * @return Axis value
	 */
	public boolean isDriverReseedButtonPressed() {
		return driveController.getOptionsButton();
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
