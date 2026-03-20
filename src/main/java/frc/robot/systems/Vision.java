package frc.robot.systems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import frc.robot.Constants.VisionConstants;
import frc.robot.generated.CommandSwerveDrivetrain;
import frc.robot.imported.LimelightHelpers;
import frc.robot.imported.LimelightHelpers.PoseEstimate;

public class Vision {
	private String limelightName;
	private VisionConsumer visionConsumer;
	private Rotation3d rotation;
	private double lastYaw = 0;
	private Pose2d lastPose = new Pose2d();
	private Drivetrain drivetrain;


	/**
	 * Construct a vision object.
	 * @param consumer The consumer to accept vision observations.
	 * @param rot The robot's rotation.
	 * @param limeLightName The name of the Limelight
	 */
	public Vision(
		VisionConsumer consumer, Rotation3d rot,
		String limeLightName, Drivetrain driv) {
		rotation = rot;
		this.limelightName = limeLightName;
		visionConsumer = consumer;
		drivetrain = driv;

		LimelightHelpers.setLEDMode_PipelineControl(limelightName);
		LimelightHelpers.SetIMUAssistAlpha(limeLightName, VisionConstants.IMU_ASSIST_ALPHA);
		LimelightHelpers.SetIMUMode(limelightName, VisionConstants.IMU_MODE);
	}

	/**
	 * Periodic method for the vision subsystem.
	 */
	public void periodic() {
		//Rotation3d rotation = rotationSupplier.get();
		LimelightHelpers.SetRobotOrientation(
			limelightName, rotation.getZ(), 0, rotation.getY(), 0, rotation.getX(), 0);

		// Angular Velocity (deg/s)
		double currentYaw = rotation.getZ();
		double angularVelocity = drivetrain.getAngularVelocity();

		// 3. Get Estimates
		PoseEstimate mt1Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
		PoseEstimate mt2Estimate =
			LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

		// Linear Velocity (m/s) - calculated from the last successful vision pose
		double linearVelocity = drivetrain.getLinearVelocityFromEncoders();

		PoseEstimate selectedEstimate = null;

		// 4. Decision Logic: Force MT2 if moving too fast (Linear or Angular)
		boolean isMovingTooFast = (angularVelocity > VisionConstants.MAX_ANGULAR_SPEED)
			|| (linearVelocity > VisionConstants.MAX_LINEAR_SPEED);

		if (isMovingTooFast) {
			if (LimelightHelpers.validPoseEstimate(mt2Estimate)) {
				selectedEstimate = mt2Estimate;
				Logger.recordOutput("Vision/ActiveMethod", "MT2 (High Speed Velocity)");
			}
		} else {
			// Normal speeds: Multi-tag MT1 is the gold standard for global localization
			if (mt1Estimate.tagCount >= 2) {
				selectedEstimate = mt1Estimate;
				Logger.recordOutput("Vision/ActiveMethod", "MT1 (Stable Multi-Tag)");
			} else if (LimelightHelpers.validPoseEstimate(mt2Estimate)) {
				selectedEstimate = mt2Estimate;
				Logger.recordOutput("Vision/ActiveMethod", "MT2 (Stable Single-Tag)");
			}
		}

		PoseEstimate visionEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
		if (LimelightHelpers.validPoseEstimate(visionEstimate) && selectedEstimate != null) {
			Pose2d pose = selectedEstimate.pose;
			lastPose = pose; //save for next velocity calc

			Logger.recordOutput("Vision/Final Pose", pose);

			if (pose.getX() > 1) {
				visionConsumer.accept(
					pose,
					visionEstimate.timestampSeconds,
					VisionConstants.LL4_STDEVS
				);
			}
		}
	}


	@FunctionalInterface
	public interface VisionConsumer {
		/**
		 * Accepts a vision observation.
		 * @param visionRobotPoseMeters The robot pose in meters.
		 * @param timestampSeconds The timestamp in seconds.
		 * @param visionMeasurementStdDevs The standard deviations of the vision
		 */
		void accept(
				Pose2d visionRobotPoseMeters,
				double timestampSeconds,
				Matrix<N3, N1> visionMeasurementStdDevs);
	}
}
