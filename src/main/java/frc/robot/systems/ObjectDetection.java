package frc.robot.systems;

import limelight.Limelight;
import limelight.networktables.LimelightResults;
import limelight.networktables.target.RetroreflectiveTape;
import frc.robot.Constants;

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
	 * Returns distance to fuel in inches.
	 * @param degreesX the degrees away from the crosshair on the x-axis
	 * @param degreesY the degrees away from the crosshair on the y-axis
	 * @return returns the distance to fuel in inches
	 */
	private Double getDistanceToFuel(double degreesX, double degreesY) {
		double radX = Math.abs(Math.toRadians(degreesX));
		double radY = Math.toRadians(degreesY);
		double multiplier = 1 / Math.cos(radX);
		double distanceY =
			Constants.LimelightConstants.LIMELIGHT_HEIGHT.in(Inches) * Math.tan(radY);
		double distance = multiplier * distanceY;
		return distance;
	}

	/**
	 * Prints the distance to the fuel.
	 */
	public void printDistanceToFuel() {
		if (checkValid()) {
			for (RetroreflectiveTape result : retroReflectives) {
				System.out.println(getDistanceToFuel(result.tx, result.ty));
			}
		}
	}
}
