package frc.robot.systems;

import limelight.Limelight;
import limelight.networktables.LimelightResults;
import limelight.networktables.target.pipeline.NeuralDetector;
import java.util.Map;
import java.util.HashMap;
import java.util.List;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Arrays;


public class LimelightBallDetectionsJSON {

	private Limelight limelight;
	private LimelightResults limelightResults;
	private NeuralDetector[] detectorResults;
	private NeuralDetector[] sortedDetectorResults;

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

	private void fetchNeuralDetectorResults() {
		if (checkValid() && limelightResults.targets_Detector != null) {
			detectorResults = limelightResults.targets_Detector;
		} else {
			detectorResults = new NeuralDetector[0];
		}
	}

	private void sortDetectorResults() {
		if (detectorResults == null || detectorResults.length == 0) {
			sortedDetectorResults = new NeuralDetector[0];
			return;
		}

		double hypotenuse;
		List<Double> hypotenuses = new ArrayList<Double>();
		Map<Double, NeuralDetector> detectors = new HashMap<Double, NeuralDetector>();

		for (NeuralDetector result : detectorResults) {
			hypotenuse = Math.sqrt(Math.pow(result.tx_pixels, 2) + Math.pow(result.ty_pixels, 2));
			hypotenuses.add(hypotenuse);
			detectors.put(hypotenuse, result);
		}

		NeuralDetector[] sortedDetectors = new NeuralDetector[hypotenuses.size()];
		Collections.sort(hypotenuses);
		int index = 0;

		for (double h : hypotenuses) {
			sortedDetectors[index] = detectors.get(h);
			index++;
		}

		sortedDetectorResults = sortedDetectors;
	}

	/**
	 * Calculates which fuel is the most optimal to pick up.
	 * @return the NeuralDetector array of the best fuel. Will return null if no targets.
	 */
	public NeuralDetector getOptimalFuel() {
		sortDetectorResults();
		if (sortedDetectorResults.length == 0) {
			return null;
		}
		return sortedDetectorResults[0];
	}

	/**
	 * Returns all detected targets sorted by proximity to center.
	 * @return sorted detector array, possibly empty.
	 */
	public NeuralDetector[] getSortedDetections() {
		sortDetectorResults();
		return Arrays.copyOf(sortedDetectorResults, sortedDetectorResults.length);
	}

	/**
	 * Updates variables. To be put in teleopPeriodic.
	 */
	public void update() {
		getLatestResults();
		fetchNeuralDetectorResults();
	}

}
