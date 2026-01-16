package frc.robot.constants;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;


public class Constants {
    public static Angle INTAKE_GROUND_TARGET = Units.Radians.of(2.6);
    public static Angle INTAKE_UPPER_TARGET = Units.Radians.of(0);
    public static final double INTAKE_KG = 0.20;
	public static final double INTAKE_KS = 0.1;
	public static final double INTAKE_KV = 0.001;
	public static final double INTAKE_KA = 0.0;
	public static final double INTAKE_KP = 3.0;
	public static final double INTAKE_KI = 0.00;
	public static final double INTAKE_KD = 0.00;

	public static final double INTAKE_CRUISE_VELO = 600;
	public static final double INTAKE_TARGET_ACCEL = 1800;
	public static final double INTAKE_EXPO_KV = 0.12;
}
  