package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Filesystem;

public class Constants {
	public static final class DriveConstants {
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

	public static final class AutoConstants {
		public static final double TRANSLATION_P = 10;
		public static final double TRANSLATION_I = 0;
		public static final double TRANSLATION_D = 0;

		public static final double ROTATION_P = 7;
		public static final double ROTATION_I = 0;
		public static final double ROTATION_D = 0;

		public static final double POSE_TOLERANCE = 0.5;
		public static final double TIME_STEP = 0.02;
		public static final double MIN_VELOCITY_VEC = 0.1;

		public static final Distance REEF_Y_LEFT_OFFSET = Inches.of(-10);
		public static final Distance REEF_Y_RIGHT_OFFSET = Inches.of(2.5);

		//TODO: These are just rough estimates: Ask what the drivers think is reasonable
		public static final Distance STATION_FAR_LEFT_OFFSET = Inches.of(-20);
		public static final Distance STATION_LEFT_OFFSET = Inches.of(-10);
		public static final Distance STATION_CENTER_OFFSET = Inches.of(0);
		public static final Distance STATION_FAR_RIGHT_OFFSET = Inches.of(10);
		public static final Distance STATION_RIGHT_OFFSET = Inches.of(20);
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

	public static final class VisionConstants {

		public static final AprilTagFieldLayout TAG_LAYOUT;

		static {
			AprilTagFieldLayout layout;
			try {
				layout = Features.USE_TEST_FIELD && !Robot.isSimulation() ? new AprilTagFieldLayout(Filesystem.
					getDeployDirectory() + "/gs-test-field.json") : AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
			} catch (IOException e) {
				System.out.println("Could not find test field, defaulting to reefscape welded field.");
				layout = AprilTagFieldLayout
				.loadField(AprilTagFields.k2025ReefscapeWelded);
			}

			var origin = new Pose3d(new Pose2d(-layout.getFieldLength() / 2, -layout.getFieldWidth() / 2, new Rotation2d()));
			layout.setOrigin(origin);
			TAG_LAYOUT = layout;

		}

		public static final int TAG_ID_TEST_STATION = 1;
		public static final int TAG_ID_TEST_REEF_LEFT = 2;
		public static final int TAG_ID_TEST_REEF_RIGHT = 3;

		public static final String REEF_CAMERA_NAME = "Reef_Camera";
		public static final String STATION_CAMERA_NAME = "Source_Camera";

		public static final Transform3d ROBOT_TO_REEF_CAM =
			new Transform3d(Units.inchesToMeters(7.129),
				-Units.inchesToMeters(4.306),
				Units.inchesToMeters(14.56), new Rotation3d(0.0, 0.0, 0.0));
		public static final Transform3d ROBOT_TO_STATION_CAM = new Transform3d(
				-Units.inchesToMeters(8.875),
				-Units.inchesToMeters(9.5),
				Units.inchesToMeters(37.596), new Rotation3d(0.0, -Math.toRadians(19), Math.PI));

		public static final double MAX_AMBIGUITY = 0.1;
		public static final double MAX_Z_ERROR = 0.3; // meters
		public static final Distance STOP_PATHFINDING_UPDATES = Meters.of(2);
		public static final Angle MAX_POSE_ROT_OFFSET = Degrees.of(5);

		public static final double LINEAR_STD_DEV_BASELINE = 0.02;
		public static final double ANGULAR_STD_DEV_BASELINE = 0.06;

		public static final double[] CAMERA_STD_DEV_FACTORS = new double[] {
			1.0, // Reef Camera
			1 // Station Camera
		};

		public static final double FIELD_BORDER_MARGIN = 0.5;
		public static final double FIELD_LENGTH = 17.5483;
		public static final double FIELD_WIDTH = 8.0519;
	}
}
