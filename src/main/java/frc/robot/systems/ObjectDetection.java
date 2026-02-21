package frc.robot.systems;

import limelight.Limelight;
import limelight.networktables.LimelightResults;
import limelight.networktables.target.RetroreflectiveTape;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;

import static edu.wpi.first.units.Units.Inches;


public class ObjectDetection {

	private Limelight limelight;
	private LimelightResults limelightResults;
	private RetroreflectiveTape[] retroReflectives;
	private RetroreflectiveTape[] sortedRetroReflectives;

	/**
	 * Constructor for LimelightBallDetectionJSON.
	 */
	public ObjectDetection() {

		limelight = new Limelight("limelight");

	}

	private void getLatestResults() {
		limelightResults = limelight.getLatestResults().orElse(null);
	}

	private boolean checkValid() {
		return limelightResults != null && limelightResults.valid;
	}

	private void fetchRetroReflectives() {
		if (checkValid() && limelightResults.targets_Detector != null) {
			retroReflectives = limelightResults.targets_Retro;
		} else {
			retroReflectives = new RetroreflectiveTape[0];
		}
	}

	/**
	 * Updates variables. To be put in teleopPeriodic.
	 */
	public void update() {
		getLatestResults();
		fetchRetroReflectives();
		printDistanceToFuel();
	}

	/**
	 * Returns raw distances (x and y).
	 * @return double list of distances.
	 */
	@AutoLogOutput(key = "Object Detection Raw Distances")
	public double[] getRawDistances() {
		if (checkValid()) {
			double radX = Math.abs(Math.toRadians(retroReflectives[0].tx));
			double radY = Math.toRadians(retroReflectives[0].ty);
			double distanceY =
				Constants.LimelightConstants.LIMELIGHT_HEIGHT.in(Inches) / Math.tan(radY);
			double distanceX = distanceY * Math.tan(radX);
			double[] distances = {distanceX, distanceY};
			return distances;
		} else {
			return new double[0];
		}
	}

	/**
	 * Returns distance to fuel in inches.
	 * @return returns the distance to fuel in inches
	 */
	public Double getDistanceToFuel() {
		if (checkValid()) {
			double radX = Math.abs(Math.toRadians(retroReflectives[0].tx));
			double radY = Math.toRadians(retroReflectives[0].ty);
			double multiplier = 1 / Math.cos(radX);
			double distanceY =
				Constants.LimelightConstants.LIMELIGHT_HEIGHT.in(Inches) / Math.tan(radY);
			double distance = multiplier * distanceY;
			return distance;
		} else {
			return null;
		}
	}

	/**
	 * Prints the distance to the fuel.
	 */
	public void printDistanceToFuel() {
		System.out.println(getRawDistances());
		System.out.println(getDistanceToFuel());
	}
}
