package frc.robot.systems;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.VisionConstants;
import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightPoseEstimator.BotPose;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.LimelightSettings.LEDMode;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;
import edu.wpi.first.math.geometry.Transform3d;

public class Vision {

    Limelight limelight;
    VisionConsumer visionConsumer;
    Supplier<Rotation3d> rotationSupplier;


    public Vision(VisionConsumer visionConsumer, Supplier<Rotation3d> rotationSupplier, String limelightName) {
        this.rotationSupplier = rotationSupplier;
        limelight = new Limelight(limelightName);
        limelight.getSettings()
             .withLimelightLEDMode(LEDMode.PipelineControl)
             .withCameraOffset(new Transform3d())
             .save();
        this.visionConsumer = visionConsumer;
    }

    public void periodic() {
        limelight.getSettings()
        .withRobotOrientation(new Orientation3d(rotationSupplier.get(), 
            new AngularVelocity3d(DegreesPerSecond.of(0),DegreesPerSecond.of(0), DegreesPerSecond.of(0))))
            .save();

        Optional<PoseEstimate> visionEstimate = BotPose.BLUE_MEGATAG2.get(limelight);
        visionEstimate.ifPresent((PoseEstimate poseEstimate) -> {
            Logger.recordOutput("Vision/MT2Pose", poseEstimate.pose.toPose2d());
            visionConsumer.accept(poseEstimate.pose.toPose2d(), poseEstimate.timestampSeconds, VisionConstants.LL4_STDEVS);
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
