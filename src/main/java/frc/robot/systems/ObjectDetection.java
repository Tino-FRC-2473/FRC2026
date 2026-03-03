package frc.robot.systems;

import limelight.Limelight;
import limelight.networktables.LimelightResults;
import limelight.networktables.target.RetroreflectiveTape;
import frc.robot.Constants;
import java.util.Arrays;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Degrees;


public class ObjectDetection {

	private Limelight limelight;
	private LimelightResults limelightResults;
	private RetroreflectiveTape[] retroReflectives;

	/**
	 * Constructor for ObjectDetection.
	 */
	public ObjectDetection() {

		limelight = new Limelight("limelight-two");

	}

	private void getLatestResults() {
		limelightResults = limelight.getLatestResults().orElse(null);
	}

	private boolean checkValid() {
		return (limelightResults != null && limelightResults.valid);
	}

	private void fetchRetroReflectives() {
		if (checkValid() && limelightResults.targets_Retro != null) {
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
	//@AutoLogOutput(key = "Object Detection Raw Distances")
	public double[] getRawDistances() {
		for (RetroreflectiveTape result : retroReflectives) {
			double radX = Math.abs(Math.toRadians(result.tx));
			double radY = Math.abs(Math.toRadians(result.ty));
			double addedY = radY + Constants.LimelightConstants.LIMELIGHT_ANGLE.in(Degrees);
			double distanceY =
				Constants.LimelightConstants.LIMELIGHT_HEIGHT.in(Inches) / Math.tan(addedY);
			double distanceX = distanceY * Math.tan(radX);
			double[] distances = {distanceX, distanceY};
			return distances;
		}

		return new double[0];
	}

	/**
	 * Returns distance to fuel in inches.
	 * @return returns the distance to fuel in inches
	 */
	public Double getDistanceToFuel() {
		for (RetroreflectiveTape result : retroReflectives) {
			double radX = (Math.toRadians(result.tx));
			double radY = (Math.toRadians(result.ty));
			double addedY =
				radY + Math.toRadians(Constants.LimelightConstants.LIMELIGHT_ANGLE.in(Degrees));
			double multiplier = 1 / Math.cos(radX);
			double distanceY =
				Constants.LimelightConstants.LIMELIGHT_HEIGHT.in(Inches) / Math.tan(addedY);
			double distance = multiplier * distanceY;
			return Math.abs(distance);
		}

		return null;
	}


	/**
	 * Prints the distance to the fuel.
	 */
	public void printDistanceToFuel() {
		if (checkValid()) {
			System.out.println(Arrays.toString(getRawDistances()));
			System.out.println(getDistanceToFuel());
		}
	}
}