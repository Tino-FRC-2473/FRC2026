package frc.robot.systems;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.VisionConstants;
import frc.robot.imported.LimelightHelpers;
import frc.robot.imported.LimelightHelpers.PoseEstimate;

public class Vision {
	private String limelightName;
	private VisionConsumer visionConsumer;
	private Rotation3d rotation;
	private Supplier<Double> angVSupplier;
	private Supplier<Double> linVSupplier;


	/**
	 * Construct a vision object.
	 * @param consumer The consumer to accept vision observations.
	 * @param rot The robot's rotation.
	 * @param limeLightName The name of the Limelight
	 * @param angVelSupp to obtain accessor methods.
	 * @param linVelSupp to obtain accessor methods.
	 */
	public Vision(
		VisionConsumer consumer, Rotation3d rot,
		String limeLightName, Supplier<Double> angVelSupp, Supplier<Double> linVelSupp) {
		rotation = rot;
		this.limelightName = limeLightName;
		visionConsumer = consumer;
		angVSupplier = angVelSupp;
		linVSupplier = linVelSupp;

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

		PoseEstimate selectedEstimate = getMegatagPose();

		if (checkIfValidPose(selectedEstimate)) {
			Pose2d pose = selectedEstimate.pose;
			Logger.recordOutput("Vision/Final Pose", pose);

			if (pose.getX() > 1) {
				visionConsumer.accept(
					pose,
					selectedEstimate.timestampSeconds,
					VisionConstants.LL4_STDEVS
				);
			}
		}
	}

	private PoseEstimate getMegatagPose() {
		// Angular Velocity (deg/s); taken from pigeon
		double angularVelocity = angVSupplier.get();
		// Linear Velocity (m/s); calculated through motor encoders
		double linearVelocity = linVSupplier.get();

		// 3. Get Estimates (will decide later which one to choose)
		PoseEstimate mt1Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
		PoseEstimate mt2Estimate =
			LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

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
		return selectedEstimate;
	}

	private boolean checkIfValidPose(PoseEstimate pose) {
		if (pose != null && pose.rawFiducials != null && pose.rawFiducials.length != 0) {
			return false;
		}

		// 1. check number of tags
		if (pose.tagCount == 0) {
			return false;
		}

		// 3. check average tag distance
		if (pose.avgTagDist > 4.0) {
			return false;
		}

		// 5. Reject if the pose is outside the actual field boundaries
		// Field is roughly 16.5m x 8.1m. Adjust constants as needed.
		if (pose.pose.getX() < 0.0 || pose.pose.getX() > 16.5
			|| pose.pose.getY() < 0.0 || pose.pose.getY() > 8.1) {
			return false;
		}

		return true;
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
