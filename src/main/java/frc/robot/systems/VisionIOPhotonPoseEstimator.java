package frc.robot.systems;

import static frc.robot.Constants.VisionConstants.TAG_LAYOUT;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Transform3d;

public class VisionIOPhotonPoseEstimator implements VisionIO {
	private final PhotonCamera camera;
	private final PhotonPoseEstimator poseEstimator;
	private final Transform3d robotToCamera;

	/**
	 * Creates a new VisionIOPhotonPoseEstimator.
	 * @param name The name of the camera.
	 * @param roboToCamera The transform from the camera to the robot.
	 */
	public VisionIOPhotonPoseEstimator(String name, Transform3d roboToCamera) {
		camera = new PhotonCamera(name);
		this.robotToCamera = roboToCamera;
		poseEstimator = new PhotonPoseEstimator(
			TAG_LAYOUT,
			PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
			robotToCamera
		);

	}

	@Override
	public void updateInputs(VisionIOInputs inputs) {
		inputs.setConnected(camera.isConnected());

		Optional<EstimatedRobotPose> estPose = Optional.empty();
		Set<Short> tagIds = new HashSet<>();
		List<PoseObservation> poseObservations = new LinkedList<>();
		for (var result : camera.getAllUnreadResults()) {
			estPose = poseEstimator.update(result);

			if (estPose.isPresent()) {
				var pose = estPose.get();

				double totalTagDistance = 0;
				for (var target : pose.targetsUsed) {
					totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
					tagIds.add((short) target.getFiducialId());
				}
				double averageTagDistance = totalTagDistance / pose.targetsUsed.size();

				poseObservations.add(
					new PoseObservation(
						result.getTimestampSeconds(),
						estPose.get().estimatedPose,
						estPose.get().targetsUsed.get(0).poseAmbiguity,
						estPose.get().targetsUsed.size(),
						averageTagDistance,
						PoseObservationType.PHOTONVISION
					)
				);
			}
		}

		inputs.setPoseObservations(new PoseObservation[poseObservations.size()]);
		for (int i = 0; i < poseObservations.size(); i++) {
			inputs.getPoseObservations()[i] = poseObservations.get(i);
		}

		inputs.setTagIds(new int[tagIds.size()]);
		int i = 0;
		for (int id : tagIds) {
			inputs.getTagIds()[i++]  = id;
		}
	}

	/**
	 * Returns the camera.
	 * @return The camera.
	 */
	public PhotonCamera getCamera() {
		return camera;
	}

	/**
	 * Returns the pose estimator.
	 * @return The pose estimator
	 */
	public PhotonPoseEstimator getPoseEstimator() {
		return poseEstimator;
	}

	/**
	 * Returns the transform from the camera to the robot.
	 * @return The transform from the camera to the robot.
	 */
	public Transform3d getRobotToCamera() {
		return robotToCamera;
	}
}
