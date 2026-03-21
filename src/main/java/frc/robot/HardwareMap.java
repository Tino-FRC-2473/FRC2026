package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * HardwareMap provides a centralized spot for constants related to the hardware
 * configuration of the robot.
 */
public final class HardwareMap {
	// ID numbers for devices on the CAN bus
	public static final int CAN_ID_CLIMBER_LEFT = 62;
	public static final int CAN_ID_CLIMBER_RIGHT = 61;
	public static final int CAN_ID_SPARK_PIVOT_LEFT = 52;
	public static final int CAN_ID_SPARK_PIVOT_RIGHT = 51;
	public static final int CAN_ID_SPARK_INTAKE = 53;

	//values TBD
	public static final int CAN_ID_FLYWHEEL = 2;
	public static final int CAN_ID_FEEDER = 3;
	public static final int CAN_ID_SPINDEXER = 4;

	//rio - dio ports
	public static final int INTAKE_GROUND_LIMIT_SWITCH_DIO_PORT = 1;
	public static final int INTAKE_TOP_LIMIT_SWITCH_DIO_PORT = 3;
	public static final int STORAGE_BREAK_BEAM_DIO_PORT = 2; //receiver of breakbeam

	// Pneumatics channel numbers
	public static final int PCM_CHANNEL_INTAKE_CYLINDER_FORWARD = 1;
	public static final int PCM_CHANNEL_INTAKE_CYLINDER_REVERSE = 2;

	// Place jumper from DIO pin 9 to GND to indicate this is a test setup
	private static final int DIO_TEST_SETUP_CHANNEL = 9;
	// Use unique DIO ports for climber limit switches to avoid collisions with intake
	public static final int CLIMBER_GROUND_LIMIT_SWITCH_DIO_PORT_LEFT = 5;
	public static final int CLIMBER_GROUND_LIMIT_SWITCH_DIO_PORT_RIGHT = 4;
	private static DigitalInput testBoardPin = new DigitalInput(HardwareMap.DIO_TEST_SETUP_CHANNEL);
	/**
	 * Check if the current RoboRIO is part of a test setup or real robot.
	 * @return true if the current setup is a test setup
	 */
	public static boolean isTestBoard() {
		return !HardwareMap.testBoardPin.get();
	}

	/**
	 * Hardware map entry for the drivetrain subsystem.
	 * @return if the hardware for the drivetrain is present
	 */
	public static boolean isDrivetrainEnabled() {
		return true;
	}

	/**
	 * Hardware map entry for the climber subsystem.
	 * @return if the hardware for the climber is present
	 */
	public static boolean isClimberEnabled() {
		return false;
	}

	/**
	 * Hardware map entry for the intake subsystem.
	 * @return if the hardware for the intake is present
	 */
	public static boolean isIntakeEnabled() {
		return true;
	}

	/**
	 * Hardware map entry for the shooter subsystem.
	 * @return if the hardware for the shooter is present
	 */
	public static boolean isShooterEnabled() {
		return false;
	}
}
