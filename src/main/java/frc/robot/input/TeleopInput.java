package frc.robot.input;
import java.util.function.Function;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import frc.robot.input.InputTypes.DoubleSignal;
import frc.robot.input.InputTypes.BooleanSignal;

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

	public static final int DRIVE_PORT = 0;
	public static final int MECH_PORT = 1;

	private PS4Controller driveController;
	private PS4Controller mechController;

	/**
	 * Constructs a TeleopInput using the constants defined in this file.
	 */
	public TeleopInput() {
		driveController = new PS4Controller(DRIVE_PORT);
		mechController = new PS4Controller(MECH_PORT);
	}

	@Override
	public Function<EventLoop, BooleanEvent> getButton(BooleanSignal key) {
		return switch (key) {

			// add / remove cases to reflect the InputTypes
			case EXAMPLE_BUTTON -> mechController::square;
			case EXAMPLE_BUTTON2 -> mechController::circle;

			default -> throw new IllegalArgumentException("Unknown button action");
		};
	}

	@Override
	public double getAxis(DoubleSignal key) {
		return switch (key) {

			// add / remove cases to reflect the InputTypes
			case DRIVE_LEFT_X -> driveController.getLeftX();
			case DRIVE_LEFT_Y -> driveController.getLeftY();
			case DRIVE_RIGHT_X -> driveController.getRightX();

			default -> throw new IllegalArgumentException("Unknown axis action");
		};
	}

}
