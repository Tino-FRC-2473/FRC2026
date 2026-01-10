package frc.robot.motors;

import edu.wpi.first.math.system.plant.DCMotor;

/**
 * Holds constants for the motor IO class such as default motor info.
 */
public class MotorConstants {
	public static final String LOGGING_PREFIX = "Motor ";
	public static final DCMotor DEFAULT_SPARK_CONFIG = DCMotor.getNeo550(1);
	public static final DCMotor DEFAULT_TALONFX_CONFIG = DCMotor.getKrakenX60(1);
}
