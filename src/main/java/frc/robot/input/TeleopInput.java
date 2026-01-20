package frc.robot.input;

// WPILib Imports
import edu.wpi.first.wpilibj.Joystick;

/**
 * Common class for providing driver inputs during Teleop.
 *
 * This class is the sole owner of WPILib input objects and is responsible for
 * polling input values. Systems may query TeleopInput via its getter methods
 * for inputs by value, but may not access the internal input objects.
 */
public final class TeleopInput implements Input {
	/* ======================== Constants ======================== */
	private static final int LEFT_JOYSTICK_PORT = 0;
	private static final int RIGHT_JOYSTICK_PORT = 1;

	/* ======================== Private variables ======================== */
	// Input objects
	private Joystick leftJoystick;
	private Joystick rightJoystick;

	/* ======================== Constructor ======================== */
	/**
	 * Create a TeleopInput and register input devices. Note that while inputs
	 * are registered at robot initialization, valid values will not be provided
	 * by WPILib until teleop mode.
	 */
	public TeleopInput() {
		leftJoystick = new Joystick(LEFT_JOYSTICK_PORT);

		rightJoystick = new Joystick(RIGHT_JOYSTICK_PORT);
	}

	/* ======================== Public methods ======================== */
	// Getter methods for fetch input values should be defined here.
	// Method names should be descriptive of the behavior, so the
	// control mapping is hidden from other classes.

	/* ------------------------ Left Joystick ------------------------ */

	@Override
	public double getLeftJoystickX() {
		return leftJoystick.getX();
	}

	@Override
	public double getLeftJoystickY() {
		return leftJoystick.getY();
	}

	@Override
	public boolean isShooterButtonPressed() {
		return leftJoystick.getRawButton(1);
	}

	@Override
	public boolean isIntakeButtonPressed() {
		return leftJoystick.getRawButton(2);
	}

	/* ------------------------ Right Joystick ------------------------ */

	@Override
	public double getRightJoystickX() {
		return rightJoystick.getX();
	}

	@Override
	public double getRightJoystickY() {
		return rightJoystick.getY();
	}

	/* ======================== Private methods ======================== */

}
