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
				System.out.println("Couldn't find test field, defaulting to reefscape welded field.");
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
	}
}