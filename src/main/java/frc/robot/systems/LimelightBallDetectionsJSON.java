package frc.robot.systems;

import limelight.Limelight;
import limelight.networktables.LimelightResults;
import limelight.networktables.target.RetroreflectiveTape;
import limelight.networktables.target.pipeline.RetroreflectiveTape;
import java.util.Map;
import java.util.Optional;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;

import java.util.HashMap;
import java.util.List;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Units;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Arrays;


public class LimelightBallDetectionsJSON {

	private Limelight limelight;
	private LimelightResults limelightResults;
	private RetroreflectiveTape[] retroReflectives;
	private RetroreflectiveTape[] sortedRetroReflectives;

	/**
	 * Constructor for LimelightBallDetectionJSON.
	 */
	public LimelightBallDetectionsJSON() {

		limelight = new Limelight("limelight");

	}

	private void getLatestResults() {
		limelightResults = limelight.getLatestResults().orElse(null);
	}

	private boolean checkValid() {
		return limelightResults != null && limelightResults.valid;
	}

	private void fetchNeuralretroReflectives() {
		if (checkValid() && limelightResults.targets_Detector != null) {
			retroReflectives = limelightResults.targets_Retro;
		} else {
			retroReflectives = new RetroreflectiveTape[0];
		}
	}

	private void sortretroReflectives() {
		if (retroReflectives == null || retroReflectives.length == 0) {
			sortedRetroReflectives = new RetroreflectiveTape[0];
			return;
		}

		double hypotenuse;
		List<Double> hypotenuses = new ArrayList<Double>();
		Map<Double, RetroreflectiveTape> detectors = new HashMap<Double, RetroreflectiveTape>();

		for (RetroreflectiveTape result : retroReflectives) {
			hypotenuse = getHypotenuse(result);
			hypotenuses.add(hypotenuse);
			detectors.put(hypotenuse, result);
		}

		RetroreflectiveTape[] sortedDetectors = new RetroreflectiveTape[hypotenuses.size()];
		Collections.sort(hypotenuses);
		int index = 0;

		for (double h : hypotenuses) {
			sortedDetectors[index] = detectors.get(h);
			index++;
		}

		sortedRetroReflectives = sortedDetectors;
	}

	/**
	 * Calculates which fuel is the most optimal to pick up.
	 * @return the RetroreflectiveTape array of the best fuel. Will return null if no targets.
	 */
	public RetroreflectiveTape getOptimalFuel() {
		sortretroReflectives();
		if (sortedRetroReflectives.length == 0) {
			return null;
		}
		return sortedRetroReflectives[0];
	}

	/**
	 * Returns all detected targets sorted by proximity to center.
	 * @return sorted detector array, possibly empty.
	 */
	public RetroreflectiveTape[] getSortedDetections() {
		sortretroReflectives();
		return Arrays.copyOf(sortedRetroReflectives, sortedRetroReflectives.length);
	}

	/**
	 * Updates variables. To be put in teleopPeriodic.
	 */
	public void update() {
		getLatestResults();
		fetchNeuralretroReflectives();
		printDistanceToFuel();
	}

	/**
	 * Gets the angle to the fuel.
	 * @return optional of the angle
	 */
	public Optional<Angle> getAngleToFuel() {
		var result = getOptimalFuel();
		if (result != null) {
			return Optional.of(Radians.of(result.tx));
		} else {
			return Optional.empty();
		}
	}

	/**
	 * Returns distance to fuel in inches.
	 * @param degreesX the degrees away from the crosshair on the x-axis
	 * @param degreesY the degrees away from the crosshair on the y-axis
	 * @return returns the distance to fuel in inches
	 */
	private Double getDistanceToFuel(double degreesX, double degreesY) {
		double multiplier = 1 / ((8100 - Math.pow(degreesX, 2)) / 90);
		double distanceX =
			Constants.LimelightConstants.LIMELIGHT_HEIGHT.in(Inches) * Math.tan(degreesX);
		double distance = multiplier * distanceX;
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

	private double getHypotenuse(RetroreflectiveTape result) {
		return Math.sqrt(Math.pow(result.tx_pixels, 2) + Math.pow(result.ty_pixels, 2));
	}

	// public Optional<Double> getDistancefromLimeToOptimalFuel() {
	// 	NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
	// 	NetworkTableEntry ty = table.getEntry("ty");
	// 	double targetOffsetAngle_Vertical = ty.getDouble(0.0);
	// 	double limelightMountAngleDegrees = getAngleToFuel();
	// }


}
