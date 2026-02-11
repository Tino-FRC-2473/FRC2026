// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.systems;


import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

import static frc.robot.Constants.VisionConstants.TAG_LAYOUT;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import org.photonvision.PhotonCamera;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {
	private final PhotonCamera camera;
	private final Transform3d robotToCamera;

	/**
	 * Creates a new VisionIOPhotonVision.
	 *
	 * @param name             The configured name of the camera.
	 * @param roboToCamera The transform from the robot to the camera.
	 */
	public VisionIOPhotonVision(String name, Transform3d roboToCamera) {
		camera = new PhotonCamera(name);
		this.robotToCamera = roboToCamera;
	}

	@Override
	public void updateInputs(VisionIOInputs inputs) {
		inputs.setConnected(camera.isConnected());

		// Read new camera observations
		Set<Short> tagIds = new HashSet<>();
		List<PoseObservation> poseObservations = new LinkedList<>();

		for (var result : camera.getAllUnreadResults()) {
			// Update latest target observation
			if (result.hasTargets()) {
				inputs.setLatestTargetObservation(
					new TargetObservation(
						Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
						Rotation2d.fromDegrees(result.getBestTarget().getPitch())
					)
				);
			} else {
				inputs.setLatestTargetObservation(
					new TargetObservation(new Rotation2d(), new Rotation2d())
				);
			}

			// Add pose observation
			if (result.multitagResult.isPresent()) { // Multitag result
				var multitagResult = result.multitagResult.get();

				// Calculate robot pose
				Transform3d fieldToCamera = multitagResult.estimatedPose.best;
				Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
				Pose3d robotPose = new Pose3d(
					fieldToRobot.getTranslation(), fieldToRobot.getRotation());

				// Calculate average tag distance
				double totalTagDistance = 0.0;
				for (var target : result.targets) {
					totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
				}

				// Add tag IDs
				tagIds.addAll(multitagResult.fiducialIDsUsed);

				// Add pose observation
				poseObservations.add(
						new PoseObservation(
								result.getTimestampSeconds(), // Timestamp
								robotPose, // 3D pose estimate
								multitagResult.estimatedPose.ambiguity, // Ambiguity
								multitagResult.fiducialIDsUsed.size(), // Tag count
								totalTagDistance / result.targets.size(), // Average tag distance
								PoseObservationType.PHOTONVISION)); // Observation type

			} else if (!result.targets.isEmpty()) { // Single tag result
				var target = result.targets.get(0);

				// Calculate robot pose
				var tagPose = TAG_LAYOUT.getTagPose(target.fiducialId);
				if (tagPose.isPresent()) {
					Transform3d fieldToTarget = new Transform3d(tagPose.get().getTranslation(),
							tagPose.get().getRotation());
					Transform3d cameraToTarget = target.bestCameraToTarget;
					Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
					Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
					Pose3d robotPose = new Pose3d(
						fieldToRobot.getTranslation(), fieldToRobot.getRotation());

					// Add tag ID
					tagIds.add((short) target.fiducialId);

					// Add observation
					poseObservations.add(
							new PoseObservation(
									result.getTimestampSeconds(), // Timestamp
									robotPose, // 3D pose estimate
									target.poseAmbiguity, // Ambiguity
									1, // Tag count
									// Average tag distance
									cameraToTarget.getTranslation().getNorm(),
									PoseObservationType.PHOTONVISION)); // Observation type
				}
			}
		}

		// Save pose observations to inputs object
		inputs.setPoseObservations(new PoseObservation[poseObservations.size()]);
		for (int i = 0; i < poseObservations.size(); i++) {
			inputs.getPoseObservations()[i] = poseObservations.get(i);
		}

		// Save tag IDs to inputs objects
		inputs.setTagIds(new int[tagIds.size()]);
		int i = 0;
		for (int id : tagIds) {
			inputs.getTagIds()[i++] = id;
		}
	}

	/**
	 * Get the camera.
	 * @return The camera.
	 */
	public PhotonCamera getCamera() {
		return camera;
	}

	/**
	 * Get the transform from the robot to the camera.
	 * @return The transform from the robot to the camera.
	 */
	public Transform3d getRobotToCamera() {
		return robotToCamera;
	}
}
