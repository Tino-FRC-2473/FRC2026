package frc.robot.input;
import java.util.function.Function;

import edu.wpi.first.wpilibj.XboxController;
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

	private final XboxController driveController;
	private final XboxController mechController;

	/**
	 * Constructs a TeleopInput using the constants defined in this file.
	 */
	public TeleopInput() {
		driveController = new XboxController(DRIVE_CONTROLLER_PORT);
		mechController = new XboxController(MECH_CONTROLLER_PORT);
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

			default -> 0.0;
		};
	}

	@Override
	public Function<EventLoop, BooleanEvent> getButtonEvent(ButtonInput key) {
		System.out.println(key);
		return switch (key) {

			// Drivetrain
			case DRIVETRAIN_RESEED -> driveController::start;
			case DRIVETRAIN_PATHFIND -> driveController::x;
			case INVERT_DRIVETRAIN_CONTROLS -> driveController::back;

			case FACE_HUB -> driveController::rightTrigger;
			case FACE_PASS -> driveController::leftTrigger;

			// Intake
			case PARTIAL_OUT_BUTTON -> mechController::start;
			case INTAKE_BUTTON -> mechController::b;
			case OUTTAKE_BUTTON -> mechController::a;
			case FOLD_IN_BUTTON -> mechController::y;
			case FOLD_OUT_BUTTON -> mechController::x;

			// Climber
			case CLIMBER_MANUAL_OVERRIDE -> mechController::povUp;
			case CLIMBER_NEXT_STEP -> mechController::povLeft;
			case CLIMBER_EMERGENCY_ABORT -> mechController::povDown;


			//Shooter
			case PASSER_PREP_TOGGLE -> mechController::rightBumper;
			case SHOOTER_PREP_TOGGLE -> mechController::leftTrigger;
			case MANUAL_SHOOT_TOGGLE -> mechController::leftBumper;
			case REV_FEEDER -> mechController::rightTrigger;
			case IDLE_SHOOTER_TOGGLE -> mechController::back;

			default -> (e) -> new BooleanEvent(e, () -> false);
		};
	}

}

