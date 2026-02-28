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
 */
public final class TeleopInput extends Input {

	public static final int DRIVE_CONTROLLER_PORT = 0;
	public static final int MECH_CONTROLLER_PORT = 1;

	private final PS4Controller driveController;
	private final PS4Controller mechController;

	/**
	 * Constructs a TeleopInput using the constants defined in this file.
	 */
	public TeleopInput() {
		driveController = new PS4Controller(DRIVE_CONTROLLER_PORT);
		mechController = new PS4Controller(MECH_CONTROLLER_PORT);
	}

	@Override
	public double getAxisValue(AxialInput key) {
		return switch (key) {

			// Drivetrain
			case DRIVETRAIN_DRIVE_Y -> driveController.getLeftX();
			case DRIVETRAIN_DRIVE_X -> driveController.getLeftY();
			case DRIVETRAIN_ROTATE -> driveController.getRightX();

			// Intake

			// Climber
			case CLIMBER_MANUAL_CONTROL -> mechController.getLeftX();

			default -> throw new IllegalArgumentException("Unknown axis input");
		};
	}

	@Override
	public Function<EventLoop, BooleanEvent> getButton(ButtonInput key) {
		return switch (key) {

			// Drivetrain
			case DRIVETRAIN_RESEED -> driveController::options;
			case FACE_HUB -> driveController::R2;

			// Intake

			//Button layout to be fixed, one idea is to make PARTIAL_OUT_BUTTON,
			//FOLD_IN_BUTTON, and FOLD_OUT_BUTTON one button that toggles between states.


			case PARTIAL_OUT_BUTTON -> mechController::share;
			case INTAKE_BUTTON -> mechController::circle;
			case OUTTAKE_BUTTON -> mechController::cross;
			case FOLD_IN_BUTTON -> mechController::L1;
			case FOLD_OUT_BUTTON -> mechController::L2;

			// Climber
			case CLIMBER_MANUAL_OVERRIDE -> mechController::triangle;
			case CLIMBER_NEXT_STEP -> mechController::square;
			case CLIMBER_EMERGENCY_ABORT -> mechController::cross;
			case CLIMBER_DOWN_BUTTON -> mechController::circle;


			case PASSER_PREP_TOGGLE -> mechController::R1;
			case SHOOTER_PREP_TOGGLE -> mechController::L1;
			case MANUAL_SHOOT_TOGGLE -> mechController::L2;
			case REV_FEEDER -> mechController::R2;
			case IDLE_SHOOTER_TOGGLE -> mechController::touchpad;

			default -> throw new IllegalArgumentException("Unknown button input");
		};
	}

}
