package frc.robot.constants;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;


public class Constants {
	public static final Angle INTAKE_GROUND_TARGET = Units.Radians.of(2.6);
	public static final Angle INTAKE_UPPER_TARGET = Units.Radians.of(0);
	public static final double INTAKE_KG = 0.20;
	public static final double INTAKE_KS = 0.1;
	public static final double INTAKE_KV = 0.001;
	public static final double INTAKE_KA = 0.0;
	public static final double INTAKE_KP = 3.0;
	public static final double INTAKE_KI = 0.00;
	public static final double INTAKE_KD = 0.00;
	public static final double INTAKE_TARGET_VELOCITY = 10.0;
	public static final double OUTTAKE_TARGET_VELOCITY = -10.0;

	//Intake Unit Conversion
	public static final double INTAKE_ROTS_TO_INCHES = 15 / (2 * Math.PI);

	public static final double INTAKE_CRUISE_VELO = 600;
	public static final double INTAKE_TARGET_ACCEL = 1800;
	public static final double INTAKE_EXPO_KV = 0.12;

	//other
	public static final int UPDATE_FREQUENCY_HZ = 100;
}
