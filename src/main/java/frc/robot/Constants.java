package frc.robot;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import java.io.IOException;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.units.Units;
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

	public static final class VisionConstants {

		public static final AprilTagFieldLayout TAG_LAYOUT;

		static {
			AprilTagFieldLayout layout;
			try {
				if (Features.USE_TEST_FIELD && !Robot.isSimulation()) {
					layout = new AprilTagFieldLayout(
						Filesystem.getDeployDirectory() + "/gs-test-field.json"
					);
				} else {
					layout = AprilTagFieldLayout.loadField(
						AprilTagFields.k2026RebuiltWelded
					);
				}
			} catch (IOException e) {
				System.out.println("Couldn't find test field, defaulting to rebuilt welded.");
				layout = AprilTagFieldLayout.loadField(
					AprilTagFields.k2026RebuiltWelded
				);
			}

			var origin = new Pose3d(new Pose2d(
				-layout.getFieldLength() / 2, -layout.getFieldWidth() / 2, new Rotation2d()
			));
			layout.setOrigin(origin);
			TAG_LAYOUT = layout;

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
}
