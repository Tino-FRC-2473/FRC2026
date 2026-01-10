package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * HardwareMap provides a centralized spot for constants related to the hardware
 * configuration of the robot.
 */
public final class HardwareMap {
	// ID numbers for devices on the CAN bus
	public static final int CAN_ID_SPARK_DRIVE_FRONT_RIGHT = 1;
	public static final int CAN_ID_SPARK_DRIVE_BACK_RIGHT = 2;
	public static final int CAN_ID_SPARK_DRIVE_FRONT_LEFT = 3;
	public static final int CAN_ID_SPARK_DRIVE_BACK_LEFT = 4;
	public static final int CAN_ID_SPARK_SHOOTER = 5;

	// Pneumatics channel numbers
	public static final int PCM_CHANNEL_INTAKE_CYLINDER_FORWARD = 1;
	public static final int PCM_CHANNEL_INTAKE_CYLINDER_REVERSE = 2;

	// Place jumper from DIO pin 9 to GND to indicate this is a test setup
	private static final int DIO_TEST_SETUP_CHANNEL = 9;
	private static DigitalInput testBoardPin = new DigitalInput(HardwareMap.DIO_TEST_SETUP_CHANNEL);
	/**
	 * Check if the current RoboRIO is part of a test setup or real robot.
	 * @return true if the current setup is a test setup
	 */
	public static boolean isTestBoard() {
		return !HardwareMap.testBoardPin.get();
	}

	/**
	 * Hardware map entry for the example FSM.
	 * @return if the hardware for the example FSM is present
	 */
	public static boolean isExampleFSMEnabled() {
		return true;
	}
}
