package frc.robot.constants;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

public class Constants {

	public static final class ClimberConstants {
		public static final double KG = 0.20;
		public static final double KS = 0.1;
		public static final double KV = 0.001;
		public static final double KA = 0.0;
		public static final double KP = 3.0;
		public static final double KI = 0.0;
		public static final double KD = 0.0;
		public static final Distance UPPER_THRESHOLD = Units
			.Inches.of(100.0);
		public static final double CRUISE_VELO = 0;
		public static final double TARGET_ACCEL = 0;
		public static final double EXPO_KV = 0;
		public static final double ROTS_TO_INCHES = 0;
		public static final Distance POSITION_TOLERANCE_L1 = Units.Inches.of(0.5);
		public static final Distance POSITION_TOLERANCE_L2_L3 = Units.Inches.of(0.5);
		public static final double JOYSTICK_DEADBAND = 0;
		public static final double MANUAL_SCALE = 0;
		// TODO: Update these positions based on actual climber design
		public static final Distance L1_EXTEND_POS = Units.Inches.of(20.0);
		public static final Distance L1_RETRACT_POS = Units.Inches.of(5.0);
		public static final Distance GROUND = Units.Inches.of(0.0);
		public static final int CONTROL_REQUEST_SUBSTRING_START_INDEX = 9;
	}
}
