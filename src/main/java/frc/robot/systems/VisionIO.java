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
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
	@AutoLog
	class VisionIOInputs {
		private boolean connected = false;
		private TargetObservation latestTargetObservation =
			new TargetObservation(new Rotation2d(), new Rotation2d());
		private PoseObservation[] poseObservations = new PoseObservation[0];
		private int[] tagIds = new int[0];

		/**
		 * Checks if the vision system is connected.
		 * @return True if the vision system is connected, false otherwise.
		 */
		public boolean isConnected() {
			return connected;
		}

		/**
		 * Sets the connection status of the vision system.
		 * @param isConnected True if the vision system is connected, false otherwise.
		 */
		public void setConnected(boolean isConnected) {
			this.connected = isConnected;
		}

		/**
		 * Gets the latest target observation.
		 * @return The latest target observation
		 */
		public TargetObservation getLatestTargetObservation() {
			return latestTargetObservation;
		}

		/**
		 * Sets the latest target observation.
		 * @param theLatestTargetObservation The latest target observation to set
		 */
		public void setLatestTargetObservation(TargetObservation theLatestTargetObservation) {
			this.latestTargetObservation = theLatestTargetObservation;
		}

		/**
		 * Get the latest pose observations.
		 * @return An array of pose observations
		 */
		public PoseObservation[] getPoseObservations() {
			return poseObservations;
		}

		/**
		 * Sets the latest pose observations.
		 * @param thePoseObservations The pose observations to set
		 */
		public void setPoseObservations(PoseObservation[] thePoseObservations) {
			this.poseObservations = thePoseObservations;
		}

		/**
		 * Gets the IDs of the AprilTags detected by the vision system.
		 * @return An array of tag IDs, which can be empty if no tags are detected.
		 */
		public int[] getTagIds() {
			return tagIds;
		}

		/**
		 * Sets the IDs of the AprilTags detected by the vision system.
		 * @param theTagIds The tag IDs to set
		 */
		public void setTagIds(int[] theTagIds) {
			this.tagIds = theTagIds;
		}
	}

	/**
	 * Represents the angle to a simple target, not used for pose estimation.
	 *
	 * @param tx The horizontal angle to the target in radians.
	 * @param ty The vertical angle to the target in radians.
	 */
	record TargetObservation(Rotation2d tx, Rotation2d ty) { }

	/**
	 * Represents a robot pose sample used for pose estimation.
	 * @param timestamp The timestamp of the observation.
	 * @param pose The pose of the robot in the field.
	 * @param ambiguity The ambiguity of the pose estimate,
	 * where 0 is no ambiguity and 1 is maximum ambiguity.
	 * @param tagCount The number of AprilTags used to estimate the pose.
	 * @param averageTagDistance The average distance to the AprilTags used to estimate the pose.
	 * @param type The type of pose observation,
	 * which can be used to differentiate between different vision systems.
	 */
	static record PoseObservation(
			double timestamp,
			Pose3d pose,
			double ambiguity,
			int tagCount,
			double averageTagDistance,
			PoseObservationType type) {
	}

	enum PoseObservationType {
		MEGATAG_1,
		MEGATAG_2,
		PHOTONVISION,
		QUESTNAV
	}

	/**
	 * Updates the inputs for the vision system.
	 * @param inputs The inputs to update.
	 */
	default void updateInputs(VisionIOInputs inputs) { }
}
