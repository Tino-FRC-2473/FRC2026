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
		PASSER_PREP_TOGGLE,
		SHOOTER_PREP_TOGGLE,
		MANUAL_SHOOT_TOGGLE,
		IDLE_SHOOTER_TOGGLE,
		REV_INDEXER
	}

}
