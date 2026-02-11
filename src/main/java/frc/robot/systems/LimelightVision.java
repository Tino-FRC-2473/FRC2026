package frc.robot.systems;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation3d;

import frc.robot.Constants.VisionConstants;

import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator.BotPose;
import limelight.networktables.LimelightSettings.LEDMode;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;

public class LimelightVision extends Vision {
	private Limelight limelight;


	/**
	 * Construct a vision object.
	 * @param consumer The consumer to accept vision observations.
	 * @param rotSupplier The supplier for the robot's rotation.
	 * @param limelightName The name of the Limelight
	 */
	public LimelightVision(
		VisionConsumer consumer, Supplier<Rotation3d> rotSupplier,
		String limelightName) {
		super(consumer, rotSupplier);
		limelight = new Limelight(limelightName);
		limelight.getSettings()
			.withLimelightLEDMode(LEDMode.PipelineControl)
			.withCameraOffset(VisionConstants.LL4_OFFSET)
			.save();
	}

	/**
	 * Periodic method for the vision subsystem.
	 */
	@Override
	public void periodic() {
		limelight.getSettings()
		.withRobotOrientation(
			new Orientation3d(
					getRotationSupplier().get(),
					new AngularVelocity3d(DegreesPerSecond.of(0),
					DegreesPerSecond.of(0), DegreesPerSecond.of(0))))
			.save();

		Optional<PoseEstimate> visionEstimate = BotPose.BLUE.get(limelight);
		visionEstimate.ifPresent((PoseEstimate poseEstimate) -> {
			Logger.recordOutput("Vision/MT2Pose", poseEstimate.pose.toPose2d());
			getVisionConsumer().accept(
					poseEstimate.pose.toPose2d(),
					poseEstimate.timestampSeconds,
					VisionConstants.LL4_STDEVS);
		});
	}
}
