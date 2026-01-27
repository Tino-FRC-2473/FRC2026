package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
// WPILib Imports
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.PS4Controller.Button;
//import edu.wpi.first.wpilibj.PS4Controller.Button;

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
	 * Get the value of the shooter button.
	 * @return True if button is pressed
	 */
	public boolean isShooterButtonPressed() {
		return leftJoystick.getRawButton(1);
	}

	/**
	 * Get the value of the intake button.
	 * @return True if button is pressed
	 */
	public boolean isIntakeButtonPressed() {
		return leftJoystick.getRawButton(2);
	}

	/* ------------------------ Right Joystick ------------------------ */
	/**
	 * Get X axis of Right Joystick.
	 * @return Axis value
	 */
	public double getRightJoystickX() {
		return rightJoystick.getX();
	}
	/**
	 * Get Y axis of Right Joystick.
	 * @return Axis value
	 */
	public double getRightJoystickY() {
		return rightJoystick.getY();
	}


	/* ======================== Teleop methods ======================== */
	/*public boolean isShooterButtonPressed() {
		return leftJoystick.getRawButton(1);
	}

	public boolean isIntakeButtonPressed() {
		return leftJoystick.getRawButton(2);
	}*/

	/**
	 * Get Boolean statement if the right bumper is pressed.
	 * @return Boolean if right bumper is pressed
	 */
	public boolean isRightBumperPressed() {
		return false;
		//rightJoystick.getRawButton(6);
		//rightJoystick.getRawButton(6);
	}

	/**
	 * Get Boolean statement if the left bumper is pressed.
	 * @return Boolean if left bumper is pressed
	 */
	public boolean isLeftBumperPressed() {

	/**
	 * Get Boolean statement if the left bumper is pressed.
	 * @return Boolean if left bumper is pressed
	 */
	public boolean isLeftBumperPressed() {
		return false;
		//rightJoystick.getRawButton(5);
		//rightJoystick.getRawButton(5);
	}

	/**
	 * Get Boolean statement if the left trigger is pressed.
	 * @return Boolean if left trigger is pressed
	 */
	public boolean isLeftTriggerPressed() {

	/**
	 * Get Boolean statement if the left trigger is pressed.
	 * @return Boolean if left trigger is pressed
	 */
	public boolean isLeftTriggerPressed() {
		return false;
		//rightJoystick.getRawButton(7);
		//rightJoystick.getRawButton(7);
	}

	/**
	 * Get Boolean statement if the right trigger is pressed.
	 * @return Boolean if right trigger is pressed
	 */
	public boolean isRightTriggerPressed() {

	/**
	 * Get Boolean statement if the right trigger is pressed.
	 * @return Boolean if right trigger is pressed
	 */
	public boolean isRightTriggerPressed() {
		return false;
		//rightJoystick.getRawButton(8);
	}

	/**
	 * Get Boolean statement if the touchpad button is pressed.
	 * @return Boolean if touchpad is pressed
	 */
	public boolean isTouchpadPressed() {
		//rightJoystick.getRawButton(8);
	}

	/**
	 * Get Boolean statement if the touchpad button is pressed.
	 * @return Boolean if touchpad is pressed
	 */
	public boolean isTouchpadPressed() {
		return false;
		//rightJoystick.getRawButton(14);
		//rightJoystick.getRawButton(14);
	}
	/* ======================== Private methods ======================== */

}
