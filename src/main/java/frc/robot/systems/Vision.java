package frc.robot.systems;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import frc.robot.Constants.VisionConstants;

// import limelight.Limelight;
// import limelight.networktables.AngularVelocity3d;
// import limelight.networktables.LimelightPoseEstimator.BotPose;
// import limelight.networktables.LimelightSettings.ImuMode;
// import limelight.networktables.LimelightSettings.LEDMode;
// import limelight.networktables.Orientation3d;
// import limelight.networktables.PoseEstimate;
import frc.robot.limelight.LimelightHelpers;
import frc.robot.limelight.LimelightHelpers.PoseEstimate;
import limelight.networktables.LimelightPoseEstimator.BotPose;

public class Vision {
	private String limelightName;
	private VisionConsumer visionConsumer;
	private Supplier<Rotation3d> rotationSupplier;


	/**
	 * Construct a vision object.
	 * @param consumer The consumer to accept vision observations.
	 * @param rotSupplier The supplier for the robot's rotation.
	 * @param limeLightName The name of the Limelight
	 */
	public Vision(
		VisionConsumer consumer, Supplier<Rotation3d> rotSupplier,
		String limeLightName) {
		rotationSupplier = rotSupplier;
		this.limelightName = limeLightName;
		visionConsumer = consumer;

		LimelightHelpers.setLEDMode_PipelineControl(limelightName);
		LimelightHelpers.SetIMUAssistAlpha(limeLightName, 0.01);
		LimelightHelpers.SetIMUMode(limelightName, 3);
	}

	/**
	 * Periodic method for the vision subsystem.
	 */
	public void periodic() {
		Rotation3d rotation = rotationSupplier.get();
		LimelightHelpers.SetRobotOrientation(limelightName, rotation.getZ(), 0, rotation.getY(), 0, rotation.getX(), 0);

		PoseEstimate visionEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

		if (LimelightHelpers.validPoseEstimate(visionEstimate)) {
            Pose2d pose = visionEstimate.pose;
            Logger.recordOutput("Vision/MT2Pose", pose);

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
