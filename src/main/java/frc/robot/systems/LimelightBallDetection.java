package frc.robot.systems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Limelight subsystem for detecting and tracking fuel balls.
 * Uses neural network detector or color-based vision pipeline
 */
public class LimelightBallDetection {

	// Limelight NetworkTable
	private NetworkTable limelightTable;

	// NetworkTable entries for vision data
	private NetworkTableEntry tx;  // Horizontal offset from crosshair to target
	private NetworkTableEntry ty;  // Vertical offset from crosshair to target
	private NetworkTableEntry ta;  // Target area (0% to 100% of image)
	private NetworkTableEntry tv;  // Whether valid target exists (0 or 1)
	private NetworkTableEntry tid; // Target ID from neural detector

	// Vision pipeline indices
	private static final int BALL_DETECTION_PIPELINE = 0;
	private static final int DRIVER_CAMERA_PIPELINE = 1;

	// LED modes
	private static final int LED_PIPELINE_DEFAULT = 0;
	private static final int LED_OFF = 1;
	private static final int LED_BLINK = 2;
	private static final int LED_ON = 3;

	/**
	 * Constructor for this class.
	 */
	public LimelightBallDetection() {
		// Get the default instance of NetworkTables
		limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

		// Get entries from the table
		tx = limelightTable.getEntry("tx");
		ty = limelightTable.getEntry("ty");
		ta = limelightTable.getEntry("ta");
		tv = limelightTable.getEntry("tv");
		tid = limelightTable.getEntry("tid");

		// Initialize with ball detection pipeline
		setPipeline(BALL_DETECTION_PIPELINE);
		setLEDMode(LED_ON);
	}

	/**
	 * Check if a valid ball target is detected.
	 * @return true if ball is detected
	 */
	public boolean hasTarget() {
		return tv.getDouble(0) == 1.0;
	}

	/**
	 * Get horizontal offset to the ball target in degrees.
	 * @return horizontal offset (-29.8 to 29.8 degrees)
	 */
	public double getHorizontalOffset() {
		return tx.getDouble(0.0);
	}

	/**
	 * Get vertical offset to the ball target in degrees.
	 * @return vertical offset (-24.85 to 24.85 degrees)
	 */
	public double getVerticalOffset() {
		return ty.getDouble(0.0);
	}

	/**
	 * Get target area percentage.
	 * @return area of target (0% to 100% of image)
	 */
	public double getTargetArea() {
		return ta.getDouble(0.0);
	}

	/**
	 * Get the class ID from neural detector.
	 * @return class ID of detected object
	 */
	public double getTargetID() {
		return tid.getDouble(0.0);
	}

	/**
	 * Calculate estimated distance to ball based on target area.
	 * This is a simplified calculation - tune constants for your setup
	 * @return estimated distance in inches
	 */
	public double getEstimatedDistance() {
		double area = getTargetArea();
		if (area <= 0) {
			return -1; // Invalid
		}

		// Example formula: distance = k / sqrt(area)
		// Tune K_DISTANCE_CONSTANT based on your camera setup and ball size
		final double kDistanceConstant = 100.0;
		return kDistanceConstant / Math.sqrt(area);
	}

	/**
	 * Check if ball is centered within tolerance.
	 * @param tolerance allowable offset in degrees
	 * @return true if ball is centered
	 */
	public boolean isBallCentered(double tolerance) {
		if (!hasTarget()) {
			return false;
		}
		return Math.abs(getHorizontalOffset()) < tolerance;
	}

	/**
	 * Get steering adjustment for aiming at ball.
	 * @param kP proportional gain for steering
	 * @return steering adjustment value (-1.0 to 1.0)
	 */
	public double getSteeringAdjustment(double kP) {
		if (!hasTarget()) {
			return 0.0;
		}

		double horizontalOffset = getHorizontalOffset();
		double steer = horizontalOffset * kP;

		// Clamp output
		if (steer > 1.0) {
			steer = 1.0;
		}
		if (steer < -1.0) {
			steer = -1.0;
		}

		return steer;
	}

	/**
	 * Set the Limelight's LED mode.
	 * @param mode LED mode (0=pipeline, 1=off, 2=blink, 3=on)
	 */
	public void setLEDMode(int mode) {
		limelightTable.getEntry("ledMode").setNumber(mode);
	}

	/**
	 * Set the Limelight's active pipeline.
	 * @param pipeline pipeline index (0-9)
	 */
	public void setPipeline(int pipeline) {
		limelightTable.getEntry("pipeline").setNumber(pipeline);
	}

	/**
	 * Set camera mode (vision processing vs driver camera).
	 * @param isVisionMode true for vision processing, false for driver camera
	 */
	public void setCameraMode(boolean isVisionMode) {
		limelightTable.getEntry("camMode").setNumber(isVisionMode ? 0 : 1);
	}

	/**
	 * Switch to driver camera mode.
	 */
	public void enableDriverMode() {
		setPipeline(DRIVER_CAMERA_PIPELINE);
		setLEDMode(LED_OFF);
		setCameraMode(false);
	}

	/**
	 * Switch to ball detection mode.
	*/
	public void enableBallDetectionMode() {
		setPipeline(BALL_DETECTION_PIPELINE);
		setLEDMode(LED_ON);
		setCameraMode(true);
	}
}
