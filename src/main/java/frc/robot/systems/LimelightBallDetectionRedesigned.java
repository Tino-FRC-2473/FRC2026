package frc.robot.systems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.Map;
import java.util.HashMap;

public class LimelightBallDetectionRedesigned {

	// Variables
	private NetworkTable limelightTable; // The table

	// Pointers
	private NetworkTableEntry tx; // X-offset from corsshair
	private NetworkTableEntry ty; // Y-offset from crosshair
	private NetworkTableEntry ta; // Target area (0-100%)
	private NetworkTableEntry tv; // Whether target is detected
	private NetworkTableEntry tid; // Target ID (for April Tags)
	private NetworkTableEntry tjson;

	// Updating Variables
	private double targetX;
	private double targetY;
	private double targetArea;
	private double targetValid;
	private double targetID; // For april tags
	private String targetJSON;

	/**
	* Constructor for object detection class.
	*/
	public LimelightBallDetectionRedesigned() {

		limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

		// Ensures that fuel is being tracked (by setting piepline)
		// Pipeline settings can be changed on the limelight 2
		if (limelightTable.getEntry("getpipe").getInteger(1) != 0) {
			limelightTable.getEntry("pipeline").setNumber(0);
		}

		// Setting pointer values
		tx = limelightTable.getEntry("tx");
		ty = limelightTable.getEntry("ty");
		ta = limelightTable.getEntry("ta");
		tv = limelightTable.getEntry("tv");
		tid = limelightTable.getEntry("tid");
		tjson = limelightTable.getEntry("json");

		// Initial update of the updating variables
		updateTargetValues();


	}

	/**
	* Periodic updater for NetworkTables and others.
	*/
	public void update() {
		updateTargetValues();
	}

	/**
	 * Updates the target values from the pointers.
	 */
	public void updateTargetValues() {
		targetX = tx.getDouble(0.0);
		targetY = ty.getDouble(0.0);
		targetArea = ta.getDouble(0.0);
		targetValid = tv.getDouble(0.0);
		targetID = tv.getDouble(0.0);
		targetJSON = tjson.getString("null");
	}

	/**
	 * Gets a list of target values.
	 * @return a list of the target values.
	 */
	public double[] getTargetValuesList() {
		updateTargetValues();
		if (targetValid != 0.0) {
			double[] valueList = new double[]{
				targetX,
				targetY,
				targetArea,
				targetValid,
				targetID
			};
			return valueList;
		} else {
			return new double[]{};
		}
	}

	/**
	 * Gets a mapping of target values to their names.
	 * @return Returns a mapping of values to their names. Returns empty map if target isn't valid.
	 */
	public Map<String, Double> getTargetValuesMap() {
		updateTargetValues();
		if (targetValid != 0.0) {
			Map<String, Double> valuesMap = Map.of(
				"targetX", targetX,
				"targetY", targetY,
				"targetArea", targetArea,
				"targetValid", targetValid,
				"targetID", targetID
			);
			return valuesMap;
		} else {
			return new HashMap<String, Double>();
		}
	}

}
