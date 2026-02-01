package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

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
	public static final class ShooterConstants {
		public static final Pose2d OUTPOST_POSE = new Pose2d();
		public static final Pose2d TARGET3_POSE = new Pose2d();
		public static final Pose2d HUB_POSE = new Pose2d();
		public static final double HOOD_GEAR_RATIO = 4;
		public static final double FLYWHEEL_GEAR_RATIO = 3;
		public static final double INDEXER_GEAR_RATIO = 3;

		public static final Frequency UPDATE_FREQUENCY_HZ = Hertz.of(200);
		public static final AngularAcceleration FLYWHEEL_ACCELERATION = DegreesPerSecondPerSecond.of(160);
		public static final double FLYWHEEL_JERK = FLYWHEEL_ACCELERATION.times(10).per(Second).magnitude();
		public static final AngularVelocity HOOD_VELOCITY = DegreesPerSecond.of(20);
		public static final AngularAcceleration HOOD_ACCELERATION = DegreesPerSecondPerSecond.of(HOOD_VELOCITY.times(2).in(DegreesPerSecond));;
		public static final double HOOD_JERK = HOOD_ACCELERATION.times(10).per(Second).magnitude();

		public static final AngularVelocity FLYWHEEL_MOE = RotationsPerSecond.of(0.5); //margin of error, subject to change
		public static final Angle HOOD_MOE = Degrees.of(0.1); //margin of error, subject to change

		public static final AngularVelocity FLYWHEEL_MAX_SPEED = RotationsPerSecond.of(160);

		public static final double FLYWHEEL_MM_CONSTANT_S = 0.1;
		//need to test by recording small amount of input that allows any movement at all
		public static final double HOOD_MM_CONSTANT_S = 0.1;
		//need to test by recording in some manner
		public static final double MM_CONSTANT_V = 0.12; //taken straight from Phoenix6
		public static final double MM_CONSTANT_A = 0.01; //taken straight from Phoenix6
		public static final Angle HOOD_MAX_ANGLE = Degrees.of(45);
		public static final Angle HOOD_MIN_ANGLE = Degrees.of(20);
		public static final Angle HOOD_INCREMENTER = Degrees.of(5);
		public static final AngularVelocity FLYWHEEL_INCREMENTER = RotationsPerSecond.of(10);
		public static final Angle FLYWHEEL_MAX_DEGREES = Degrees.of(360);

		//All of these are placeholder values, all need to be changed
		public static final double FLYWHEEL_MM_CONSTANT_P = 0;
		public static final double FLYWHEEL_MM_CONSTANT_I = 0;
		public static final double FLYWHEEL_MM_CONSTANT_D = 0;
		public static final double HOOD_MM_CONSTANT_P = 0;
		public static final double HOOD_MM_CONSTANT_I = 0;
		public static final double HOOD_MM_CONSTANT_D = 0;
		public static final double HOOD_MM_CONSTANT_G = 0.82;
		//just an estimate, will use Recalc with measurements to calculate later
	}
}
