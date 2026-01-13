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
	private static final int DRIVER_CONTROLLER_PORT = 0;

	/* ======================== Private variables ======================== */
	// Input objects
	private PS4Controller driverController;

	/* ======================== Constructor ======================== */
	/**
	 * Create a TeleopInput and register input devices. Note that while inputs
	 * are registered at robot initialization, valid values will not be provided
	 * by WPILib until teleop mode.
	 */
	public TeleopInput() {
		driverController = new PS4Controller(DRIVER_CONTROLLER_PORT);
	}

	/* ======================== Public methods ======================== */
	// Getter methods for fetch input values should be defined here.
	// Method names should be descriptive of the behavior, so the
	// control mapping is hidden from other classes.

	/* ------------------------ Driver Controller ------------------------ */
	/**
	 * Get X axis of Left Joystick.
	 * @return Axis value
	 */
	public double getDriverLeftX() {
		return driverController.getLeftX();
	}
	/**
	 * Get Y axis of Left Joystick.
	 * @return Axis value
	 */
	public double getDriverLeftY() {
		return driverController.getLeftY();
	}

	/**
	 * Get X axis of Right Joystick.
	 * @return Axis value
	 */
	public double getDriverRightX() {
		return driverController.getRightX();
	}
	/**
	 * Get Y axis of Right Joystick.
	 * @return Axis value
	 */
	public double getDriverRightY() {
		return driverController.getRightY();
	}

	/* ======================== Private methods ======================== */

}
