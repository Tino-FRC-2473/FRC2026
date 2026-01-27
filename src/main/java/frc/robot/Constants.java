package frc.robot;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.math.geometry.Pose2d;

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
	public static final class ShooterConstants {
		public static final Pose2d OUTPOST_POSE = new Pose2d();
		public static final Pose2d TARGET3_POSE = new Pose2d();
		public static final Pose2d HUB_POSE = new Pose2d();
		public static final double HOOD_GEAR_RATIO = 4;
		public static final double FLYWHEEL_GEAR_RATIO = 3;
		public static final double INDEXER_GEAR_RATIO = 3;

		public static final int UPDATE_FREQUENCY_HZ = 200;
		public static final double FLYWHEEL_ACCELERATION = 160;
		public static final double FLYWHEEL_JERK = 10 * FLYWHEEL_ACCELERATION;
		public static final double HOOD_VELOCITY = 20;
		public static final double HOOD_ACCELERATION = 2 * HOOD_VELOCITY;
		public static final double HOOD_JERK = 10 * HOOD_ACCELERATION;

		public static final double FLYWHEEL_MOE = 0.5; //margin of error, subject to change
		public static final double HOOD_MOE = 0.1; //margin of error, subject to change

		public static final double FLYWHEEL_MAX_SPEED = 160;
		//rotations/second, temporary/placeholder

		public static final double FLYWHEEL_MM_CONSTANT_S = 0.1;
		//need to test by recording small amount of input that allows any movement at all
		public static final double HOOD_MM_CONSTANT_S = 0.1;
		//need to test by recording in some manner
		public static final double MM_CONSTANT_V = 0.12; //taken straight from Phoenix6
		public static final double MM_CONSTANT_A = 0.01; //taken straight from Phoenix6
		public static final double FLYWHEEL_MAX_ANGLE = 70;
		public static final double FLYWHEEL_MIN_ANGLE = 45;
		public static final double HOOD_INCREMENTER = 5;
		public static final double FLYWHEEL_INCREMENTER = 10;

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
