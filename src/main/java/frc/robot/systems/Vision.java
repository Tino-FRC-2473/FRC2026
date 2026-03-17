package frc.robot.systems;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.imported.LimelightHelpers;
import frc.robot.imported.LimelightHelpers.PoseEstimate;

public class Vision {
	private String limelightName;
	private VisionConsumer visionConsumer;
	private Rotation3d rotation;
	private VisionSystemSim visionSim;
	private PhotonCameraSim cameraSim;


	/**
	 * Construct a vision object.
	 * @param consumer The consumer to accept vision observations.
	 * @param rot The robot's rotation.
	 * @param limeLightName The name of the Limelight
	 */
	public Vision(
		VisionConsumer consumer, Rotation3d rot,
		String limeLightName) {
		rotation = rot;
		this.limelightName = limeLightName;
		visionConsumer = consumer;

		LimelightHelpers.setLEDMode_PipelineControl(limelightName);
		LimelightHelpers.SetIMUAssistAlpha(limeLightName, VisionConstants.IMU_ASSIST_ALPHA);
		LimelightHelpers.SetIMUMode(limelightName, VisionConstants.IMU_MODE);

		if (RobotBase.isSimulation()) {
			visionSim = new VisionSystemSim("main");
			AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
			visionSim.addAprilTags(tagLayout);
			SimCameraProperties cameraProp = new SimCameraProperties();
			cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));

			PhotonCamera camera = new PhotonCamera("SimCamera");
			cameraSim = new PhotonCameraSim(camera, cameraProp);
			cameraSim.enableRawStream(true);
			cameraSim.enableProcessedStream(true);

			Transform3d robotToCamera = new Transform3d(
			new Translation3d(0.1, 0, 0.5),
			new Rotation3d(0, Math.toRadians(-15), 0)
			);

			visionSim.addCamera(cameraSim, robotToCamera);
		}
	}
	/**
	 * Update the vision simulation with the robot's pose.
	 * @param robotPoseMeters
	 */
	public void updateSim(Pose2d robotPoseMeters) {
		if (visionSim != null) {
			visionSim.update(robotPoseMeters);
			var cameraPoseOpt = visionSim.getCameraPose(cameraSim);
			if (cameraPoseOpt.isPresent()) {
				var cameraPose3d = cameraPoseOpt.get();
				var cameraPose2d = new Pose2d(
					cameraPose3d.getTranslation().toTranslation2d(),
					cameraPose3d.getRotation().toRotation2d()
				);
				Logger.recordOutput("Vision/SimCameraPose", cameraPose2d);
			}
		}
	}

	/**
	 * Periodic method for the vision subsystem.
	 */
	public void periodic() {
		//Rotation3d rotation = rotationSupplier.get();
		LimelightHelpers.SetRobotOrientation(
			limelightName, rotation.getZ(), 0, rotation.getY(), 0, rotation.getX(), 0);

		PoseEstimate visionEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
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
