package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.MomentOfInertia;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.KilogramSquareMeters;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;

import static edu.wpi.first.units.Units.Seconds;



public class Constants {
	public static final class LimelightConstants {
		public static final long AUTO_UPDATE_INTERVAL_MS = 20L;
	}

	public static final class AutoPathsConstants {
		public static final double SHOOT_CLIMB_SECONDS = 10;
		public static final double SHOOT_WAIT_TIME_SECONDS = 1.35;
	}

	public static final class DrivetrainConstants {
		public static final int NUM_MODULES = 4;
		public static final double SYS_ID_VOLT_DAMP = 6;

		public static final double TRANSLATION_DEADBAND = 0.05;
		public static final double ROTATION_DEADBAND = 0.05;


		//Set to the decimal corresponding to the percentage of how fast you want the bot to go
		// 1 = 100% speed, 0.5 = 50% speed, 0.3 = 30% speed, and so on
		public static final double TRANSLATIONAL_DAMP = 1;
		public static final double ROTATIONAL_DAMP = 1;

		// drive velocity limits
		// Max linear & angular speeds
		public static final LinearVelocity MAX_SPEED = TunerConstants.kSpeedAt12Volts;
		public static final AngularVelocity MAX_ANGULAR_SPEED = RotationsPerSecond.of(1.5);

		// pathing acceleration limits
		public static final LinearAcceleration MAX_PATHING_LINEAR_ACCELERATION
			= MetersPerSecondPerSecond.of(4);

		public static final AngularAcceleration MAX_PATHING_ROTATIONAL_ACCELERATION
			= RotationsPerSecondPerSecond.of(4);

		public static final int PIGEON2_CAN_ID = 1;
		public static final String CAN_BUS_NAME = "Drivetrain";
		//TODO: Get some actual values for this
		public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(
				MAX_SPEED,
				MAX_PATHING_LINEAR_ACCELERATION,
				MAX_ANGULAR_SPEED,
				MAX_PATHING_ROTATIONAL_ACCELERATION,
				Voltage.ofBaseUnits(12, Volts)
			);

		//new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);


		public static final AngularVelocity MAX_ANGULAR_VELOCITY = RotationsPerSecond.of(0.75);
		public static final int SWERVE_MODULE_COUNT = 4;

		// Drivetrain deadbands
		public static final double TRANSLATIONAL_DEADBAND = 0.1;
		public static final double ROTATIONAL_DEADBAND = 0.1;

		private static int tagToAlignTo;

		/**
		 * @return the tag to align to
		 */
		public static int getTagToAlignTo() {
			return tagToAlignTo;
		}

		/**
		 * @param newTagToAlignTo the new value to set
		 */
		public static void setTagToAlignTo(int newTagToAlignTo) {
			DrivetrainConstants.tagToAlignTo = newTagToAlignTo;
		}

		public static final double X_TRANFORM_FROM_TAG = -1;
		public static final double Y_TRANFORM_FROM_TAG = 0;
		public static final double FACE_HUB_P = 7.0;
		public static final double FACE_HUB_I = 0.0;
		public static final double FACE_HUB_D = 0.0;

		public static final Pose2d RED_HUB_POSE =
			new Pose2d(11.9191774, 4.0346376, new Rotation2d());
		public static final Pose2d RED_OUTPOST_POSE =
			new Pose2d(8.2741742, 2.0346376, new Rotation2d());
		public static final Pose2d BLUE_HUB_POSE =
			new Pose2d(14.001, 4.0346376, new Rotation2d());
		public static final Pose2d BLUE_OUTPOST_POSE =
			new Pose2d(2.54, 2.0346376, new Rotation2d());
		public static final Pose2d RED_POSE3_POSE = new Pose2d(2.54, 6.0346376, new Rotation2d());
		public static final Pose2d BLUE_POSE3_POSE =
			new Pose2d(8.2741742, 6.0346376, new Rotation2d());
	}

	public static final class ModuleConstants {
		public static final double DRIVE_P = 7;
		public static final double DRIVE_I = 0.1;
		public static final double DRIVE_D = 0.4;
		public static final double DRIVE_V = 0.124;

		public static final double DRIVE_CURRENT_LIMIT = 80;
		public static final double STEER_CURRENT_LIMIT = 60;

		public static final double STEER_P = 7;
		public static final double STEER_I = 0.1;
		public static final double STEER_D = 0.4;
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
		public static final double CRUISE_VELO = 30; // 30 max
		public static final double TARGET_ACCEL = 15; // 30 max
		public static final double EXPO_KV = 0;
		public static final double ROTS_TO_INCHES = 16 / 105.5;
		public static final Distance POSITION_TOLERANCE_L1 = Units.Inches.of(0.4);
		public static final Distance DOWN_POSITION_TOLERANCE_L1 = Units.Inches.of(0.05);
		public static final double JOYSTICK_DEADBAND = 0.1;
		public static final double MANUAL_SCALE = 1;
		public static final Distance L1_EXTEND_POS = Units.Inches.of(17.0);
		public static final Distance L1_RETRACT_POS = Units.Inches.of(6.0);
		public static final Distance GROUND = Units.Inches.of(-2.0);
		public static final Distance REVERSE_LIMIT_SWITCH_POS = Units.Inches.of(-20.0);
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
		public static final Transform3d ROBOT_TO_LAUNCHER =
				new Transform3d(-0.276, 0.09, 0.599, new Rotation3d(0.0, 0.0, Math.PI));

		public static final Pose2d OUTPOST_POSE = new Pose2d(8.2741742, 2.0346376, new Rotation2d());
		public static final Pose2d TARGET3_POSE = new Pose2d(8.2741742, 6.0346376, new Rotation2d());
		public static final Pose2d HUB_POSE = new Pose2d(11.9191774, 4.0346376, new Rotation2d());
		//public static final double HOOD_GEAR_RATIO = 4;
		public static final double SPINDEX_GEAR_RATIO = 3;
		public static final double FLYWHEEL_GEAR_RATIO = 1;
		public static final double FEEDER_GEAR_RATIO = 3;

		public static final double SHOOTER_CURRENT_LIMIT = 40; //Amps

		public static final double TEMP_FLYSPEED = 50;

		private static final double JERK_MULT_CONSTANT = 100;
		//constant to change the magnitude of jerk from acceleration
		public static final Frequency UPDATE_FREQUENCY_HZ = Hertz.of(200);
		public static final AngularAcceleration MAGIC_ACCELERATION =
			RotationsPerSecondPerSecond.of(10000);
		public static final double MAGIC_JERK =
			MAGIC_ACCELERATION.times(JERK_MULT_CONSTANT).per(Seconds).magnitude();
		// public static final AngularVelocity HOOD_VELOCITY = DegreesPerSecond.of(20);
		// public static final AngularAcceleration HOOD_ACCELERATION =
		// 	DegreesPerSecondPerSecond.of(HOOD_VELOCITY.times(2).in(RotationsPerSecond));
		// public static final double HOOD_JERK =
		// 	HOOD_ACCELERATION.times(JERK_MULT_CONSTANT).per(Seconds).magnitude();

		public static final AngularVelocity FLYWHEEL_MOE =
			RotationsPerSecond.of(3); //margin of error, subject to change
		// public static final Angle HOOD_MOE = Degrees.of(0.1);
		//margin of error, subject to change

		public static final AngularVelocity FLYWHEEL_MAX_SPEED = RotationsPerSecond.of(160);

		public static final double FLYWHEEL_MM_CONSTANT_S = 0; // 0.1;
		//need to test by recording small amount of input that allows any movement at all
		// public static final double HOOD_MM_CONSTANT_S = 0.1;n
		//need to test by recording in some manner
		public static final double MM_CONSTANT_V = 0.112; //taken straight from Phoenix6
		// public static final Angle HOOD_MAX_ANGLE = Degrees.of(45);
		// public static final Angle HOOD_MIN_ANGLE = Degrees.of(20);
		// public static final Angle HOOD_INCREMENTER = Degrees.of(5);
		public static final Angle HOOD_ANGLE = Degrees.of(30); //from vertical

		public static final AngularVelocity FLYWHEEL_INCREMENTER = RotationsPerSecond.of(10);
		public static final Angle FLYWHEEL_MAX_DEGREES = Degrees.of(360);

		public static final double FEEDER_MM_CONSTANT_S = 0.1;
		public static final double FEEDER_MM_CONSTANT_P = 0;
		public static final double FEEDER_MM_CONSTANT_I = 0;
		public static final double FEEDER_MM_CONSTANT_D = 0;
		public static final double FEEDER_CONSTANT_SPEED = 20;

		//All of these are placeholder values, all need to be changed
		public static final double FLYWHEEL_MM_CONSTANT_P = 0.8;
		public static final double FLYWHEEL_MM_CONSTANT_I = 0.02;
		public static final double FLYWHEEL_MM_CONSTANT_D = 0;
		// public static final double HOOD_MM_CONSTANT_P = 0;
		// public static final double HOOD_MM_CONSTANT_I = 0;
		// public static final double HOOD_MM_CONSTANT_D = 0;
		// public static final double HOOD_MM_CONSTANT_G = 0.82;
		//just an estimate, will use Recalc with measurements to calculate later

		public static final double SPINDEX_CONSTANT_SPEED = 0.1; //ranges from -1 to 1
		public static final double SPINDEX_CONSTANT_VOLTAGE = 12;

		public static final double FEED_MAX_TIME = 2; //2s

		public static final double PASSING_REGRESSION_CONSTANT = 15.8;
		public static final double PASSING_REGRESSION_SLOPE = 0.233; //inches

		public static final double SPINDEX_MAX_TIME = 2.0; //3s, needs to be tuned
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

	public static final class VisionConstants {
		public static final int RESOLUTION_X = 640;
		public static final int RESOLUTION_Y = 480;
		public static final double IMU_ASSIST_ALPHA = 0.01;
		public static final int IMU_MODE = 3;
		public static final String LIMELIGHT_NAME = "limelight-four";
		//TODO: Find some actual values for this.
		public static final Matrix<N3, N1> LL4_STDEVS = VecBuilder.fill(
			0.02, 0.02, Math.toRadians(1));
		//TODO: Measure this on the bot
		public static final Pose3d LL4_OFFSET =
			new Pose3d(0.096, -0.03, 0.55, new Rotation3d(0, 0, 270));
	}
}
