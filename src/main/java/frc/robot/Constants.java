package frc.robot;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

public class Constants {
	public static final class DrivetrainConstants {
		public static final int NUM_MODULES = 4;
		public static final double SYS_ID_VOLT_DAMP = 6;

		public static final double TRANSLATION_DEADBAND = 0.1;
		public static final double ROTATION_DEADBAND = 0.1;
		public static final AngularVelocity MAX_ANGULAR_VELO_RPS = RotationsPerSecond.of(0.75);

		//Set to the decimal corresponding to the percentage of how fast you want the bot to go
		// 1 = 100% speed, 0.5 = 50% speed, 0.3 = 30% speed, and so on
		public static final double TRANSLATIONAL_DAMP = 1;
		public static final double ROTATIONAL_DAMP = 1;
	}

	public static final class ModuleConstants {
		public static final double DRIVE_P = 0.1;
		public static final double DRIVE_I = 0;
		public static final double DRIVE_D = 0;
		public static final double DRIVE_V = 0.124;

		public static final double DRIVE_CURRENT_LIMIT = 60;
		public static final double STEER_CURRENT_LIMIT = 60;

		public static final double STEER_P = 100;
		public static final double STEER_I = 0;
		public static final double STEER_D = 0.5;
		public static final double STEER_V = 0.1;
		public static final double STEER_S = 0;
	}

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
		public static final Distance L1_EXTEND_POS = Units.Inches.of(20.0);
		public static final Distance L1_RETRACT_POS = Units.Inches.of(5.0);
		public static final Distance GROUND = Units.Inches.of(0.0);
		public static final int CONTROL_REQUEST_SUBSTRING_START_INDEX = 9;
	}

	public static final class IntakeConstants {
		//Targets for Pivot
		public static final Angle INTAKE_GROUND_TARGET = Units.Radians.of(2.09);
		public static final Angle INTAKE_UPPER_TARGET = Units.Radians.of(0);
		public static final Angle PARTIAL_OUT_TARGET = Units.Radians.of(0.79);

		public static final double PIVOT_MAX_ROTATION = 2.09;
		public static final double PIVOT_MIN_ROTATION = 0;

		//Arm length in meters
		public static final double PIVOT_ARM_LENGTH = 0.5;

		//The moment of inertia of the arm in kg-m²; can be calculated from CAD software.
		public static final double J = 0.1;

		//Pivot PID
		public static final double PIVOT_KG = 0.35;
		public static final double PIVOT_KS = 0.25;
		public static final double PIVOT_KV = 0.12;
		public static final double PIVOT_KA = 0.01;
		public static final double PIVOT_KP = 5.0;
		public static final double PIVOT_KI = 0.0;
		public static final double PIVOT_KD = 0.2;

		//Intake Motor PID
		public static final double INTAKE_KV = 0.12;
		public static final double INTAKE_KA = 0.0;
		public static final double INTAKE_KP = 0.15;
		public static final double INTAKE_KI = 0.0;
		public static final double INTAKE_KD = 0.0;
		public static final double INTAKE_TARGET_VELOCITY = 20;
		public static final double OUTTAKE_TARGET_VELOCITY = -25.0;

		//Intake Gearing/Velocity Factors
		public static final double INTAKE_PIVOT_GEARING = 62.5 / (2 * Math.PI);
		public static final double INTAKE_GEARING = 3 / (2 * Math.PI);

		public static final double PIVOT_CRUISE_VELO = 20;
		public static final double PIVOT_TARGET_ACCEL = 60;
		public static final double PIVOT_EXPO_KV = 0.35;

		public static final double INTAKE_CRUISE_VELO = 7;
		public static final double INTAKE_TARGET_ACCEL = 20;
		public static final double INTAKE_EXPO_KV = 0.12;

		//other
		public static final int UPDATE_FREQUENCY_HZ = 100;
	}
}
