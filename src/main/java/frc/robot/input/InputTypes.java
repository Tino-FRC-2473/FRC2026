package frc.robot.input;

public class InputTypes {

	// Inputs that come in the form of an axis value
	public enum AxialInput {
		// Drivetrain
		DRIVETRAIN_DRIVE_X,
		DRIVETRAIN_DRIVE_Y,
		DRIVETRAIN_ROTATE,

		// Intake

		// Climber
		CLIMBER_MANUAL_CONTROL
	}

	// Inputs that come in the form of a button press
	public enum ButtonInput {
		// Drivetrain
		DRIVETRAIN_RESEED,

		// Intake
		INTAKE_FOLD_IN,
		INTAKE_FOLD_OUT,
		INTAKE_PARTIAL_OUT,
		INTAKE_INTAKE,
		INTAKE_OUTTAKE,

		// Climber
		CLIMBER_MANUAL_OVERRIDE,
		CLIMBER_NEXT_STEP,
		CLIMBER_EMERGENCY_ABORT,
		CLIMBER_DOWN_BUTTON
	}

}
