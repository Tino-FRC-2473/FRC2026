package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
// WPILib Imports
import edu.wpi.first.wpilibj.Joystick;
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
	private static final int LEFT_JOYSTICK_PORT = 0;
	private static final int RIGHT_JOYSTICK_PORT = 1;
	private static final int CONTROLLER_PORT = 0;

	/* ======================== Private variables ======================== */
	// Input objects
	private Joystick leftJoystick;
	private Joystick rightJoystick;
	private PS4Controller controller;
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
		leftJoystick = new Joystick(LEFT_JOYSTICK_PORT);

		rightJoystick = new Joystick(RIGHT_JOYSTICK_PORT);
		controller = new PS4Controller(CONTROLLER_PORT);
	}

	/**
	 * Getter for the fold in button being pressed.
	 * @return whether the fold in button was pressed
	 */
	public boolean isFoldInButtonPressed() {
		return controller.getCircleButtonPressed();
	}

	/**
	 * Getter for the fold out button being pressed.
	 * @return whether the fold out button was pressed
	 */
	public boolean isFoldOutButtonPressed() {
		return controller.getCircleButtonPressed();
	}

	/**
	 * Getter for the partial out button being pressed.
	 * @return whether the partial out button was pressed
	 */
	public boolean isPartialOutButtonPressed() {
		return controller.getOptionsButtonPressed();
	}

	/**
	 * Getter for the intake button being pressed.
	 * @return whether the intake button was pressed
	 */
	public boolean isIntakeButtonPressed() {
		return controller.getTriangleButtonPressed();
	}

	/**
	 * Getter for the intake button being released.
	 * @return whether the intake button was released
	 */
	public boolean isIntakeButtonReleased() {
		return controller.getTriangleButtonReleased();
	}

	/**
	 * Getter for the outtake button being pressed.
	 * @return whether the outtake button was pressed
	 */
	public boolean isOuttakeButtonPressed() {
		return controller.getSquareButtonPressed();
	}

	/**
	 * Getter for the outtake button being released.
	 * @return whether the outtake button was released
	 */
	public boolean isOuttakeButtonReleased() {
		return controller.getSquareButtonReleased();
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
	/* ------------------------ Right Joystick ------------------------ */

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

	/* ======================== Private methods ======================== */

}
