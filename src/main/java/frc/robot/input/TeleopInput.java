package frc.robot.input;
import java.util.function.Function;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import frc.robot.input.InputTypes.AxialInput;
import frc.robot.input.InputTypes.ButtonInput;

/**
 * Common class for providing driver inputs during Teleop.
 *
 *
 * This class is the sole owner of WPILib input objects and is responsible for
 * polling input values. Systems may query TeleopInput via its getter methods
 * for inputs by value, but may not access the internal input objects.
 *
 */
public final class TeleopInput extends Input {

	public static final int DRIVE_CONTROLLER_PORT = 0;
	public static final int MECH_CONTROLLER_PORT = 1;

	private PS4Controller driveController;
	private PS4Controller mechController;

	/**
	 * Constructs a TeleopInput using the constants defined in this file.
	 */
	public TeleopInput() {
		driveController = new PS4Controller(DRIVE_CONTROLLER_PORT);
		mechController = new PS4Controller(MECH_CONTROLLER_PORT);
	}

	/**
	 * Getter for the fold in button being pressed.
	 * @return whether the fold in button was pressed
	 */
	public boolean isFoldInButtonPressed() {
		return mechController.getCircleButtonPressed();
	}

	/**
	 * Getter for the fold out button being pressed.
	 * @return whether the fold out button was pressed
	 */
	public boolean isFoldOutButtonPressed() {
		return mechController.getCircleButtonPressed();
	}

	/**
	 * Getter for the partial out button being pressed.
	 * @return whether the partial out button was pressed
	 */
	public boolean isPartialOutButtonPressed() {
		return mechController.getOptionsButtonPressed();
	}

	/**
	 * Getter for the intake button being pressed.
	 * @return whether the intake button was pressed
	 */
	public boolean isIntakeButtonPressed() {
		return mechController.getTriangleButtonPressed();
	}

	/**
	 * Getter for the intake button being released.
	 * @return whether the intake button was released
	 */
	public boolean isIntakeButtonReleased() {
		return mechController.getTriangleButtonReleased();
	}

	/**
	 * Getter for the outtake button being pressed.
	 * @return whether the outtake button was pressed
	 */
	public boolean isOuttakeButtonPressed() {
		return mechController.getSquareButtonPressed();
	}

	/**
	 * Getter for the outtake button being released.
	 * @return whether the outtake button was released
	 */
	public boolean isOuttakeButtonReleased() {
		return mechController.getSquareButtonReleased();
	}


	@Override
	public Function<EventLoop, BooleanEvent> getButton(ButtonInput key) {
		return switch (key) {

			// add / remove cases to reflect the InputTypes
			case RESEED_DRIVETRAIN -> mechController::options;
			case CLIMBER_MANUAL_OVERRIDE -> mechController::triangle;
			case CLIMBER_NEXT_STEP -> mechController::square;
			case CLIMBER_EMERGENCY_ABORT -> mechController::R1;
			case CLIMBER_DOWN_BUTTON -> mechController::R2;

			default -> throw new IllegalArgumentException("Unknown button action");
		};
	}

	@Override
	public double getAxis(AxialInput key) {
		return switch (key) {

			// add / remove cases to reflect the InputTypes
			case DRIVE_Y -> driveController.getLeftX();
			case DRIVE_X -> driveController.getLeftY();
			case ROTATE -> driveController.getRightX();
			case CLIMBER_MANUAL_CONTROL -> mechController.getLeftX();

			default -> throw new IllegalArgumentException("Unknown axis action");
		};
	}


}
