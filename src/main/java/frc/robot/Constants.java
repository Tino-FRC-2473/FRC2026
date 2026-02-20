package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.MomentOfInertia;
import static edu.wpi.first.units.Units.KilogramSquareMeters;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Seconds;


public class Constants {
	public static final class LimelightConstants {
		public static final long AUTO_UPDATE_INTERVAL_MS = 20L;
	}

	public static final class DrivetrainConstants {
		// Speed controls
		// Decimal value corresponding to a percentage of max speed
		// 1.0 = 100% speed, 0.5 = 50% speed, etc.
		public static final double TRANSLATIONAL_DAMP = 1;
		public static final double ROTATIONAL_DAMP = 1;

		public static final AngularVelocity MAX_ANGULAR_VELOCITY = RotationsPerSecond.of(0.75);
		public static final int SWERVE_MODULE_COUNT = 4;
		public static final double SYS_ID_VOLT_DAMP = 6;

		// Drivetrain deadbands
		public static final double TRANSLATIONAL_DEADBAND = 0.1;
		public static final double ROTATIONAL_DEADBAND = 0.1;
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
		public static final double CRUISE_VELO = 5;
		public static final double TARGET_ACCEL = 5;
		public static final double EXPO_KV = 0;
		public static final double ROTS_TO_INCHES = 0;
		public static final Distance POSITION_TOLERANCE_L1 = Units.Inches.of(0.5);
		public static final Distance POSITION_TOLERANCE_L2_L3 = Units.Inches.of(0.5);
		public static final double JOYSTICK_DEADBAND = 0.1;
		public static final double MANUAL_SCALE = 0.5;
		public static final Distance L1_EXTEND_POS = Units.Inches.of(20.0);
		public static final Distance L1_RETRACT_POS = Units.Inches.of(5.0);
		public static final Distance GROUND = Units.Inches.of(-1.0);
		public static final int CONTROL_REQUEST_SUBSTRING_START_INDEX = 9;

		public static final double CLIMBER_ANGLE_RAD = Math.toRadians(48.0);
		public static final double CLIMBER_GEAR_RATIO = 9.0;
		public static final double CLIMBER_WEIGHT_LBS = 15.0;
		public static final double UPDATE_RATE = 0.02;
		public static final double LIMIT_SWITCH_HEIGHT = 0.01;
		public static final double EFFECTIVE_WEIGHT = edu.wpi.first.math.util.Units.lbsToKilograms(
			ClimberConstants.CLIMBER_WEIGHT_LBS)
			* Math.sin(ClimberConstants.CLIMBER_ANGLE_RAD
			);
		public static final double DRUM_CIRCUMFERENCE_METERS = edu.wpi.first.math.util.Units
			.inchesToMeters(1.0) * 2 * Math.PI;
	}
	public static final class ShooterConstants {
		public static final Pose2d OUTPOST_POSE = new Pose2d();
		public static final Pose2d TARGET3_POSE = new Pose2d();
		public static final Pose2d HUB_POSE = new Pose2d();
		//public static final double HOOD_GEAR_RATIO = 4;
		public static final double SPINDEX_GEAR_RATIO = 3;
		public static final double FLYWHEEL_GEAR_RATIO = 3;
		public static final double FEEDER_GEAR_RATIO = 3;
	}
	public static final class IntakeConstants {

		public static final double INTAKE_PIVOT_GEARING = 62.5;
		public static final double INTAKE_GEARING = 3;

		//Targets for Pivot
		public static final Angle UPPER_TARGET_ANGLE = Units.Radians.of(2.2);
		public static final Angle GROUND_TARGET_ANGLE = Units.Radians.of(0);
		public static final Angle PARTIAL_OUT_TARGET_ANGLE = Units.Radians.of(1.5);

		public static final double PIVOT_MAX_ROTATION = 2.09;
		public static final double PIVOT_MIN_ROTATION = 0;

		//Arm length in meters
		public static final double PIVOT_ARM_LENGTH = 0.5;

		//The moment of inertia of the arm in kg-m²; can be calculated from CAD software.
		public static final MomentOfInertia J = KilogramSquareMeters.of(0.1);

		//Pivot PID
		public static final double PIVOT_KG = 0.2;
		public static final double PIVOT_KS = 0.2;  //0.5
		public static final double PIVOT_KV = 0.06;
		public static final double PIVOT_KA = 0.03;
		public static final double PIVOT_KP = 25;  //0.1
		public static final double PIVOT_KI = 0.0;
		public static final double PIVOT_KD = 0.2;

		//Intake Motor PID
		public static final double INTAKE_KV = 0.12;
		public static final double INTAKE_KA = 0.0;
		public static final double INTAKE_KP = 0.2;
		public static final double INTAKE_KI = 0.0;
		public static final double INTAKE_KD = 0.0;
		public static final double INTAKE_TARGET_VELOCITY = 40;
		public static final double OUTTAKE_TARGET_VELOCITY = -40.0;

		//Intake Gearing/Velocity Factors
		public static final double PIVOT_BUFFER = 0.01;
		public static final double PIVOT_CRUISE_VELO = 15;
		public static final double PIVOT_TARGET_ACCEL = 30;
		public static final double PIVOT_EXPO_KV = 0.12;

		public static final double INTAKE_CRUISE_VELO = 20;
		public static final double INTAKE_TARGET_ACCEL = 40;
		public static final double INTAKE_EXPO_KV = 0.12;

		//other
		public static final double PIVOT_CURRENT_LIMIT = 10; //in amps
		public static final Frequency UPDATE_FREQUENCY = Units.Hertz.of(100);
		public static final double SIM_UPDATE_SECONDS = 0.02;
		public static final Angle SIM_LIMIT_SWITCH_BUFFER = Units.Radians.of(0.01);
	}
}
