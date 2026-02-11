package frc.robot;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import java.nio.file.Path;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;

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

	public class IntakeConstants {
		// Target angles
		public static final Angle GROUND_TARGET_ANGLE = Units.Radians.of(2.09);
		public static final Angle UPPER_TARGET_ANGLE = Units.Radians.of(0);
		public static final Angle PARTIAL_OUT_TARGET_ANGLE = Units.Radians.of(0.79);

		// Target velocities
		public static final AngularVelocity INTAKE_TARGET_VELOCITY = Units.RotationsPerSecond
				.of(20);
		public static final AngularVelocity OUTTAKE_TARGET_VELOCITY = Units.RotationsPerSecond
				.of(-25.0);

		// Gearing ratios
		public static final double PIVOT_GEARING = 62.5 / (2 * Math.PI);
		public static final double INTAKE_GEARING = 3.0 / (2 * Math.PI);

		// Motion magic constants
		public static final double PIVOT_CRUISE_VELOCITY = 20;
		public static final double PIVOT_ACCELERATION = 60;
		public static final double PIVOT_EXPO_KV = 0.35;

		public static final double INTAKE_CRUISE_VELOCITY = 7;
		public static final double INTAKE_ACCELERATION = 20;
		public static final double INTAKE_EXPO_KV = 0.12;

		// PID values
		public static final double PIVOT_G = 0.35;
		public static final double PIVOT_S = 0.25;
		public static final double PIVOT_V = 0.12;
		public static final double PIVOT_A = 0.01;
		public static final double PIVOT_P = 5.0;
		public static final double PIVOT_I = 0.0;
		public static final double PIVOT_D = 0.2;
		public static final double INTAKE_G = 0.0;
		public static final double INTAKE_S = 0.20;
		public static final double INTAKE_V = 0.12;
		public static final double INTAKE_A = 0.0;
		public static final double INTAKE_P = 0.15;
		public static final double INTAKE_I = 0.0;
		public static final double INTAKE_D = 0.0;

		// Update frequency
		public static final Frequency UPDATE_FREQUENCY = Units.Hertz.of(100);

		public static final int PIGEON2_CAN_ID = 1;
		public static final String CAN_BUS_NAME = "Drivetrain";
		//TODO: Get some actual values for this
		public static final PathConstraints PATH_CONSTRAINTS = PathConstraints.unlimitedConstraints(12);

		//new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);

	}

	public static final class ModuleConstants {
		public static double DRIVE_P = 0.4;
		public static final double DRIVE_I = 0;
		public static final double DRIVE_D = 0.1;
		public static final double DRIVE_V = 0.124;

		public static final double DRIVE_CURRENT_LIMIT = 60;
		public static final double STEER_CURRENT_LIMIT = 60;

		public static double STEER_P = 0.4;
		public static final double STEER_I = 0;
		public static final double STEER_D = 0.1;
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


	public static final class VisionConstants {
		public static final String LIMELIGHT_NAME = "limelight-four";
		//TODO: Find some actual values for this.
		public static final Matrix<N3, N1> LL4_STDEVS = VecBuilder.fill(
			0.02, 0.02, Math.toRadians(1));
		//TODO: Measure this on the bot
		public static final Pose3d LL4_OFFSET = new Pose3d(0.096, -0.03, 0.55, new Rotation3d(0, 0, 270));
	}
}
