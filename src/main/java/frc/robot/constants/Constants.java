package frc.robot.constants;

import static edu.wpi.first.units.Units.Degree;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class Constants {
	public static final double CLIMBER_KG = 0.20;
	public static final double CLIMBER_KS = 0.1;
	public static final double CLIMBER_KV = 0.001;
	public static final double CLIMBER_KA = 0.0;
	public static final double CLIMBER_KP = 3.0;
	public static final double CLIMBER_KI = 0.0;
	public static final double CLIMBER_KD = 0.0;
	public static final Distance CLIMBER_UPPER_THRESHOLD = Units
		.Inches.of(100.0);
	public static final double CLIMBER_CRUISE_VELO = 0;
	public static final double CLIMBER_TARGET_ACCEL = 0;
	public static final double CLIMBER_EXPO_KV = 0;
	public static final double CLIMBER_ROTS_TO_INCHES = 0;
	public static final double CLIMBER_TILT_ROTS_TO_ANGLE = 0;
	public static final double CLIMBER_TILT_KG = 0;
	public static final double CLIMBER_TILT_KS = 0;
	public static final double CLIMBER_TILT_KV = 0;
	public static final double CLIMBER_TILT_KA = 0;
	public static final double CLIMBER_TILT_KP = 0;
	public static final double CLIMBER_TILT_KI = 0;
	public static final double CLIMBER_TILT_KD = 0;
	public static final double CLIMBER_TILT_CRUISE_VELO = 0;
	public static final double CLIMBER_TILT_TARGET_ACCEL = 0;
	public static final double CLIMBER_TILT_EXPO_KV = 0;
	public static final double CLIMBER_POSITION_TOLERANCE_L1 = 0;
	public static final double CLIMBER_POSITION_TOLERANCE_L2_L3 = 0;
	public static final double CLIMBER_JOYSTICK_DEADBAND = 0;
	public static final double CLIMBER_MANUAL_SCALE = 0;
	public static final Angle CLIMBER_TILT_EXTEND_POS = Angle.ofRelativeUnits(45, Degree);
	public static final Angle CLIMBER_TILT_RETRACT_POS = Angle.ofRelativeUnits(0, Degree);
	// TODO: Update these positions based on actual climber design
	public static final double L1_EXTEND_POS = 20.0;
	public static final double L1_RETRACT_POS = 5.0;
	public static final double GROUND = 0.0;
}
