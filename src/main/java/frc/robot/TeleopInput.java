package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
// WPILib Imports

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

	/* ======================== Private variables ======================== */
	// Input objects
	private PS4Controller driveController;

	/* ======================== Constructor ======================== */
	/**
	 * Create a TeleopInput and register input devices. Note that while inputs
	 * are registered at robot initialization, valid values will not be provided
	 * by WPILib until teleop mode.
	 */
	public TeleopInput() {
		driveController = new PS4Controller(DRIVE_CONTROLLER_PORT);
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
	}
	/**
	 * Get Y axis of the left joystick.
	 * @return Axis value
	 */
	public double getDriverLeftY() {
		return driveController.getLeftY();
	}

	/**
	 * Get X axis of the right joystick.
	 * @return Axis value
	 */
	public double getDriverRightX() {
		return driveController.getRightX();
	}
	/**
	 * Get Y axis of the right joystick.
	 * @return Axis value
	 */
	public double getDriverRightY() {
		return driveController.getRightY();
	}

	/**
	 * Get share button state for drive controller.
	 * @return Axis value
	 */
	public boolean isDriverReseedButtonPressed() {
		return driveController.getOptionsButton();
	}

	/* ========================== Triggers =============================*/
	public boolean isLeftBumperPressed(){
		return false;
	}
	public boolean isLeftTriggerPressed(){
		return false;
	}
	public boolean isRightBumperPressed(){
		return false;
	}
	public boolean isRightTriggerPressed(){
		return false;
	}
	/*============================= Touchpad ========================== */
	public boolean isTouchpadPressed(){
		return false;
	}


	/* ======================== Private methods ======================== */

}
