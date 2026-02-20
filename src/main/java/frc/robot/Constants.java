package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;

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
		// public static final double KG = 0.15;
		// public static final double KS = 0.08;
		// public static final double KV = 0.00;
		// public static final double KA = 0.0;
		// public static final double KP = 0.7;
		// public static final double KI = 0.0;
		// public static final double KD = 0.05;
		public static final double KG = 0.05;
		public static final double KS = 0.05;
		public static final double KV = 0.00;
		public static final double KA = 0.0;
		public static final double KP = 5.0;
		public static final double KI = 0.0;
		public static final double KD = 0.0;
		public static final Distance UPPER_THRESHOLD = Units
			.Inches.of(17);
		public static final double CRUISE_VELO = 30;
		public static final double TARGET_ACCEL = 30;
		public static final double EXPO_KV = 0;
		public static final double ROTS_TO_INCHES = 16 / 105.5;
		public static final Distance POSITION_TOLERANCE_L1 = Units.Inches.of(0.4);
		public static final Distance DOWN_POSITION_TOLERANCE_L1 = Units.Inches.of(0.05);
		public static final Distance POSITION_TOLERANCE_L2_L3 = Units.Inches.of(0.5);
		public static final double JOYSTICK_DEADBAND = 0.1;
		public static final double MANUAL_SCALE = 1;
		public static final Distance L1_EXTEND_POS = Units.Inches.of(10.0);
		public static final Distance L1_RETRACT_POS = Units.Inches.of(5.0);
		public static final Distance GROUND = Units.Inches.of(-1.0);
		public static final int CONTROL_REQUEST_SUBSTRING_START_INDEX = 9;

		public static final double CLIMBER_ANGLE_RAD = Math.toRadians(48.0);
		public static final double CLIMBER_GEAR_RATIO = 7.2;
		public static final double CLIMBER_WEIGHT_KGS = 1.36078;
		public static final double UPDATE_RATE = 0.02;
		public static final double LIMIT_SWITCH_HEIGHT = 0.01;
		public static final double EFFECTIVE_WEIGHT = CLIMBER_WEIGHT_KGS;
		public static final double DRUM_CIRCUMFERENCE_METERS = edu.wpi.first.math.util.Units
			.inchesToMeters(0.75) * Math.PI;
	}
	public static final class ShooterConstants {
		public static final Pose2d OUTPOST_POSE = new Pose2d();
		public static final Pose2d TARGET3_POSE = new Pose2d();
		public static final Pose2d HUB_POSE = new Pose2d();
		//public static final double HOOD_GEAR_RATIO = 4;
		public static final double SPINDEX_GEAR_RATIO = 3;
		public static final double FLYWHEEL_GEAR_RATIO = 3;
		public static final double FEEDER_GEAR_RATIO = 3;

		public static final double SHOOTER_CURRENT_LIMIT = 50; //Amps

		public static final double TEMP_FLYSPEED = 20;

		private static final double JERK_MULT_CONSTANT = 10;
		//constant to change the magnitude of jerk from acceleration
		public static final Frequency UPDATE_FREQUENCY_HZ = Hertz.of(200);
		public static final AngularAcceleration MAGIC_ACCELERATION =
			RotationsPerSecondPerSecond.of(160);
		public static final double MAGIC_JERK =
			MAGIC_ACCELERATION.times(JERK_MULT_CONSTANT).per(Seconds).magnitude();
		// public static final AngularVelocity HOOD_VELOCITY = DegreesPerSecond.of(20);
		// public static final AngularAcceleration HOOD_ACCELERATION =
		// 	DegreesPerSecondPerSecond.of(HOOD_VELOCITY.times(2).in(RotationsPerSecond));
		// public static final double HOOD_JERK =
		// 	HOOD_ACCELERATION.times(JERK_MULT_CONSTANT).per(Seconds).magnitude();

		public static final AngularVelocity FLYWHEEL_MOE =
			RotationsPerSecond.of(0.5); //margin of error, subject to change
		// public static final Angle HOOD_MOE = Degrees.of(0.1);
		//margin of error, subject to change

		public static final AngularVelocity FLYWHEEL_MAX_SPEED = RotationsPerSecond.of(160);

		public static final double FLYWHEEL_MM_CONSTANT_S = 0.1;
		//need to test by recording small amount of input that allows any movement at all
		// public static final double HOOD_MM_CONSTANT_S = 0.1;
		//need to test by recording in some manner
		public static final double MM_CONSTANT_V = 0.112; //taken straight from Phoenix6
		// public static final Angle HOOD_MAX_ANGLE = Degrees.of(45);
		// public static final Angle HOOD_MIN_ANGLE = Degrees.of(20);
		// public static final Angle HOOD_INCREMENTER = Degrees.of(5);
		public static final Angle HOOD_ANGLE = Degrees.of(45);
		public static final AngularVelocity FLYWHEEL_INCREMENTER = RotationsPerSecond.of(10);
		public static final Angle FLYWHEEL_MAX_DEGREES = Degrees.of(360);

		public static final double FEEDER_MM_CONSTANT_S = 0.1;
		public static final double FEEDER_MM_CONSTANT_P = 0.1;
		public static final double FEEDER_MM_CONSTANT_I = 0;
		public static final double FEEDER_MM_CONSTANT_D = 0;

		//All of these are placeholder values, all need to be changed
		public static final double FLYWHEEL_MM_CONSTANT_P = 0;
		public static final double FLYWHEEL_MM_CONSTANT_I = 0;
		public static final double FLYWHEEL_MM_CONSTANT_D = 0;
		// public static final double HOOD_MM_CONSTANT_P = 0;
		// public static final double HOOD_MM_CONSTANT_I = 0;
		// public static final double HOOD_MM_CONSTANT_D = 0;
		// public static final double HOOD_MM_CONSTANT_G = 0.82;
		//just an estimate, will use Recalc with measurements to calculate later

		public static final double SPINDEX_CONSTANT_SPEED = 0.1; //ranges from -1 to 1
		public static final double SPINDEX_CONSTANT_VOLTAGE = 1;

		public static final double FEED_MAX_TIME = 2; //2s
	}
	public static final class IntakeConstants {
		//Targets for Pivot
		public static final Angle GROUND_TARGET_ANGLE = Units.Radians.of(2.09);
		public static final Angle UPPER_TARGET_ANGLE = Units.Radians.of(0);
		public static final Angle PARTIAL_OUT_TARGET_ANGLE = Units.Radians.of(0.79);

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
		public static final Frequency UPDATE_FREQUENCY = Units.Hertz.of(100);
		public static final double SIM_UPDATE_SECONDS = 0.02;
		public static final Angle SIM_LIMIT_SWITCH_BUFFER = Units.Radians.of(0.01);
	}
}
