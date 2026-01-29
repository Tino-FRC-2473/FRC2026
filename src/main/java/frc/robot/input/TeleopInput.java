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


	@Override
	public Function<EventLoop, BooleanEvent> getButton(ButtonInput key) {
		return switch (key) {

			// add / remove cases to reflect the InputTypes
			case RESEED_DRIVETRAIN -> mechController::options;
			case CLIMBER_MANUAL_OVERRIDE -> mechController::triangle;
			case CLIMBER_NEXT_STEP -> mechController::square;
			case CLIMBER_EMERGENCY_ABORT -> mechController::R1;
			case CLIMBER_DOWN_BUTTON -> mechController::R2;
			case PARTIAL_OUT_BUTTON -> mechController::options;
			case INTAKE_BUTTON -> mechController::triangle;
			case OUTTAKE_BUTTON -> mechController::square;
			case FOLD_IN_BUTTON -> mechController::circle;
			case FOLD_OUT_BUTTON -> mechController::cross;

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
