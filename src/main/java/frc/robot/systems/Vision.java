package frc.robot.systems;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import frc.robot.Constants.VisionConstants;

import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator.BotPose;
import limelight.networktables.LimelightSettings.LEDMode;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;

public class Vision {
	private Limelight limelight;
	private VisionConsumer visionConsumer;
	private Supplier<Rotation3d> rotationSupplier;


	/**
	 * Construct a vision object.
	 * @param consumer The consumer to accept vision observations.
	 * @param rotSupplier The supplier for the robot's rotation.
	 * @param limelightName The name of the Limelight
	 */
	public Vision(
		VisionConsumer consumer, Supplier<Rotation3d> rotSupplier,
		String limelightName) {
		rotationSupplier = rotSupplier;
		limelight = new Limelight(limelightName);
		limelight.getSettings()
			.withLimelightLEDMode(LEDMode.PipelineControl)
			.withCameraOffset(VisionConstants.LL4_OFFSET)
			.save();
		visionConsumer = consumer;
	}

	/**
	 * Periodic method for the vision subsystem.
	 */
	public void periodic() {
		limelight.getSettings()
		.withRobotOrientation(
			new Orientation3d(
					rotationSupplier.get(),
					new AngularVelocity3d(DegreesPerSecond.of(0),
					DegreesPerSecond.of(0), DegreesPerSecond.of(0))))
			.save();

		Optional<PoseEstimate> visionEstimate = BotPose.BLUE_MEGATAG2.get(limelight);
		visionEstimate.ifPresent((PoseEstimate poseEstimate) -> {
			Logger.recordOutput("Vision/MT2Pose", poseEstimate.pose.toPose2d());
			visionConsumer.accept(
					poseEstimate.pose.toPose2d(),
					poseEstimate.timestampSeconds,
					VisionConstants.LL4_STDEVS);
		});
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
