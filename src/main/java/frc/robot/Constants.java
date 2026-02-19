package frc.robot;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.math.Matrix;

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

	public static final class SimConstants {
		public static final Mass MASS_WITH_BUMPER = Pounds.of(115);
		public static final Distance ROBOT_LENGTH = Inches.of(34.5);
		public static final Distance ROBOT_WIDTH = Inches.of(34.5);
		public static final double WHEEL_COF = 1.2;

		public static final double STEER_P = 70;
		public static final double STEER_I = 0;
		public static final double STEER_D = 4.5;
		public static final double STEER_S = 0;
		public static final double STEER_V = 1.91;
		public static final double STEER_A = 0;

		public static final double STEER_MOTOR_GEAR_RATIO = 16.0;
		public static final Voltage DRIVE_FRICTION_VOLTAGE = Volts.of(0.1);
		public static final Voltage STEER_FRICTION_VOLTAGE = Volts.of(0.51);
		public static final MomentOfInertia STEER_INERTIA = KilogramSquareMeters.of(0.05);

		/* ================== PhotonSim Camera Properties ==================== */
		public static final int CAM_FPS = 100;
		public static final int CAM_RES_WIDTH_PIX = 640;
		public static final int CAM_RES_HEIGHT_PIX = 480;


		/* THE FOLLOWING CAMERA PROPERTIES ARE TAKEN FROM THE camprops.sqlite FILE */
		public static final Matrix<N3, N3> REEF_CAMERA_CALIBRATION = new Matrix<>(
			N3.instance, N3.instance,
				new double[] {
					554.8363329613238,
					0.0,
					319.771006175582,
					0.0,
					555.7640379607542,
					210.90231168898111,
					0.0,
					0.0,
					1.0
				}
			);

		public static final Matrix<N8, N1> REEF_CAMERA_DISTORTION = new Matrix<>(
			N8.instance, N1.instance,
				new double[] {
					0.032904169887820925,
					0.024981667114235325,
					-0.0024512685439365967,
					9.347928373666906E-4,
					-0.15993971100687385,
					-2.8908154357146817E-4,
					1.516375932970693E-4,
					0.006735034604041476
				}
			);

		public static final Matrix<N3, N3> STATION_CAMERA_CALIBRATION = new Matrix<>(
			N3.instance, N3.instance,
				new double[] {
					548.8107781815636,
					0.0,
					335.98845208944647,
					0.0,
					549.91022315822,
					261.5076314193876,
					0.0,
					0.0,
					1.0
				}
			);

		public static final Matrix<N8, N1> STATION_CAMERA_DISTORTION = new Matrix<>(
			N8.instance, N1.instance,
				new double[] {
					0.046882076180144325,
					-0.08739491623632688,
					-7.369602850193537E-4,
					9.49279422750342E-4,
					0.015437967521711683,
					-0.0018478126980591776,
					0.004435053264404992,
					-1.8178696218760975E-4
				}
			);
	}

}
