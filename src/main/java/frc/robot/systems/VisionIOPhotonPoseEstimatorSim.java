package frc.robot.systems;

import static frc.robot.Constants.VisionConstants.TAG_LAYOUT;

import java.util.function.Supplier;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionIOPhotonPoseEstimatorSim extends VisionIOPhotonPoseEstimator {
	private static VisionSystemSim visionSystem;
	private final Supplier<Pose2d> poseSupplier;
	private final PhotonCameraSim cameraSim;

	/**
	 * Creates a new VisionIOPhotonPoseEstimatorSim.
	 * @param name the name of the camera
	 * @param robotToCamera the transform from the robot to the camera
	 * @param pose2dSupplier a supplier for the robot's pose
	 */
	public VisionIOPhotonPoseEstimatorSim(
		String name,
		Transform3d robotToCamera,
		Supplier<Pose2d> pose2dSupplier
	) {
		super(name, robotToCamera);
		this.poseSupplier = pose2dSupplier;

		if (visionSystem == null) {
			visionSystem = new VisionSystemSim("main");
			visionSystem.addAprilTags(TAG_LAYOUT);
		}

		var camProps = new SimCameraProperties();
		cameraSim = new PhotonCameraSim(getCamera(), camProps, TAG_LAYOUT);
		visionSystem.addCamera(cameraSim, robotToCamera);
	}

	@Override
	public void updateInputs(VisionIOInputs inputs) {
		visionSystem.update(poseSupplier.get());
		super.updateInputs(inputs);
	}
}
