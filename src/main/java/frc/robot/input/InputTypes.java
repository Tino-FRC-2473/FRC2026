package frc.robot.input;

public class InputTypes {

	// enum of axes, change this as needed
	// make sure to change TeleopInput to support the new input type
	public enum DoubleSignal {
		DRIVE_LEFT_X,
		DRIVE_RIGHT_X,
		DRIVE_LEFT_Y
	}

	// enum of buttons, change this as needed
	// make sure to change TeleopInput to support the new input type
	public enum BooleanSignal {
		DRIVE_RESEED;
	}

}
