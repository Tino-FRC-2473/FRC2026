package frc.robot.systems;

import java.util.function.Supplier;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public abstract class Vision {
	private VisionConsumer visionConsumer;
	private Supplier<Rotation3d> rotationSupplier;


	/**
	 * Construct a vision object.
	 * @param consumer The consumer to accept vision observations.
	 * @param rotSupplier The supplier for the robot's rotation.
	 */
	public Vision(
		VisionConsumer consumer, Supplier<Rotation3d> rotSupplier) {
		rotationSupplier = rotSupplier;
		visionConsumer = consumer;
	}

	/**
	 * Periodic method for the vision subsystem.
	 */
	abstract void periodic();

	/**
	 * Getter for the visonConsumer.
	 * @return visionconsumer
	 */
	public VisionConsumer getVisionConsumer() {
		return visionConsumer;
	}

	/**
	 * Getter for the rotationSupplier.
	 * @return rotationSupplier
	 */
	public Supplier<Rotation3d> getRotationSupplier() {
		return rotationSupplier;
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
