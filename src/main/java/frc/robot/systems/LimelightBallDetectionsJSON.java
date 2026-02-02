package frc.robot.systems;

import limelight.Limelight;
import limelight.networktables.LimelightResults;
import limelight.networktables.target.pipeline.NeuralDetector;
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

		NeuralDetector[] sortedDetectors = Arrays.copyOf(detectorResults, detectorResults.length);
		Arrays.sort(
			sortedDetectors,
			(left, right) -> Double.compare(
				Math.hypot(left.tx_pixels, left.ty_pixels),
				Math.hypot(right.tx_pixels, right.ty_pixels)
			)
		);
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
