package frc.robot.input;

public class InputTypes {

	// change this as needed & update TeleopInput to support new input types
	public enum AxialInput {
		DRIVE_Y,
		ROTATE,
		DRIVE_X,
		CLIMBER_MANUAL_CONTROL
	}

	// change this as needed & update TeleopInput to support new input types
	public enum ButtonInput {
		RESEED_DRIVETRAIN,
		CLIMBER_MANUAL_OVERRIDE,
		CLIMBER_NEXT_STEP,
		CLIMBER_EMERGENCY_ABORT,
		CLIMBER_DOWN_BUTTON,
		FOLD_IN_BUTTON,
		FOLD_OUT_BUTTON,
		PARTIAL_OUT_BUTTON,
		INTAKE_BUTTON,
		OUTTAKE_BUTTON
	}

}
