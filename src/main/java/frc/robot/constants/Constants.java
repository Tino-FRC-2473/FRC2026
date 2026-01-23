package frc.robot.constants;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;


public class Constants {
	public static final Angle INTAKE_GROUND_TARGET = Units.Radians.of(2.09);
	public static final Angle INTAKE_UPPER_TARGET = Units.Radians.of(0);
	public static final double PIVOT_KG = 0.35;
	public static final double PIVOT_KS = 0.25;
	public static final double PIVOT_KV = 0.12;
	public static final double PIVOT_KA = 0.01;
	public static final double PIVOT_KP = 5.0;
	public static final double PIVOT_KI = 0.0;
	public static final double PIVOT_KD = 0.2;
	public static final double INTAKE_KG = 0.0;
	public static final double INTAKE_KS = 0.20;
	public static final double INTAKE_KV = 0.12;
	public static final double INTAKE_KA = 0.0;
	public static final double INTAKE_KP = 0.15;
	public static final double INTAKE_KI = 0.0;
	public static final double INTAKE_KD = 0.0;
	public static final double INTAKE_TARGET_VELOCITY = 20;
	public static final double OUTTAKE_TARGET_VELOCITY = -25.0;
	public static final Angle PARTIAL_OUT_POSITION = Units.Radians.of(0.79);

	//Intake Unit Conversion
	public static final double INTAKE_ROTS_TO_INCHES = 15 / (2 * Math.PI);

	public static final double INTAKE_CRUISE_VELO = 600;
	public static final double INTAKE_TARGET_ACCEL = 1800;
	public static final double INTAKE_EXPO_KV = 0.12;

	//other
	public static final int UPDATE_FREQUENCY_HZ = 100;
}
