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
		DRIVETRAIN_PATHFIND,
		ALIGN_TO_BALL,

		// Climber
		CLIMBER_MANUAL_OVERRIDE,
		CLIMBER_NEXT_STEP,
		CLIMBER_EMERGENCY_ABORT,
		CLIMBER_DOWN_BUTTON,

		//Shooter
		PASSER_PREP_TOGGLE,
		SHOOTER_PREP_TOGGLE,
		MANUAL_SHOOT_TOGGLE,
		IDLE_SHOOTER_TOGGLE,
		REV_FEEDER,
		// Intake
		INTAKE_BUTTON,
		OUTTAKE_BUTTON,
		FOLD_IN_BUTTON,
		FOLD_OUT_BUTTON,
		PARTIAL_OUT_BUTTON
	}

}
