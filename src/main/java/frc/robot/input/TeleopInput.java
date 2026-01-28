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
	public Function<EventLoop, BooleanEvent> getButton(ButtonInput key) {
		return switch (key) {

			// add / remove cases to reflect the InputTypes
			case RESEED_DRIVETRAIN -> mechController::options;

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

			default -> throw new IllegalArgumentException("Unknown axis action");
		};
	}

}
