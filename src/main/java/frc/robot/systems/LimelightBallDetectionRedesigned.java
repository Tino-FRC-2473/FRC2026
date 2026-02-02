package frc.robot.systems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import java.util.Map;
import java.util.logging.Logger;

/**
 * Provides access to Limelight target data for ball detection.
 */
public class LimelightBallDetectionRedesigned {

    private static final Logger LOGGER = Logger.getLogger(LimelightBallDetectionRedesigned.class.getName());

    // Variables
    private NetworkTable limelightTable; // The table

    // Pointers
    private NetworkTableEntry tx; // X-offset from crosshair
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

    private double lastTargetValid = -1.0;

    private boolean autoUpdate = true;
    private long lastAutoUpdateMs = 0L;
    private long autoUpdateIntervalMs = Constants.LimelightConstants.AUTO_UPDATE_INTERVAL_MS;

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
        targetID = tid.getDouble(0.0);
        targetJSON = tjson.getString("null");

        if (targetValid != lastTargetValid) {
            LOGGER.info("Limelight target valid changed: " + targetValid);
            lastTargetValid = targetValid;
        }
        if (targetValid != 0.0) {
            LOGGER.fine(
                "Limelight target: x=" + targetX
                + " y=" + targetY
                + " area=" + targetArea
                + " id=" + targetID
            );
        }
    }

    /**
     * Enables or disables automatic NetworkTables updates in getters.
     * @param enabled true to auto-update on getter calls.
     */
    public void setAutoUpdate(boolean enabled) {
        autoUpdate = enabled;
    }

    /**
     * Returns whether automatic updates are enabled.
     * @return true if auto-update is enabled.
     */
    public boolean isAutoUpdateEnabled() {
        return autoUpdate;
    }

    /**
     * Sets the minimum interval between automatic updates.
     * @param intervalMs minimum interval in milliseconds
     */
    public void setAutoUpdateIntervalMs(long intervalMs) {
        autoUpdateIntervalMs = Math.max(0L, intervalMs);
    }

    /**
     * Returns the minimum interval between automatic updates.
     * @return minimum interval in milliseconds
     */
    public long getAutoUpdateIntervalMs() {
        return autoUpdateIntervalMs;
    }

    /**
     * Updates target values if auto-update is enabled.
     */
    private void updateIfAuto() {
        if (!autoUpdate) {
            return;
        }
        long nowMs = System.currentTimeMillis();
        if (nowMs - lastAutoUpdateMs >= autoUpdateIntervalMs) {
            updateTargetValues();
            lastAutoUpdateMs = nowMs;
        }
    }

    /**
     * Gets a list of target values.
     * @return a list of the target values.
     */
    public double[] getTargetValuesList() {
        updateIfAuto();
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
        updateIfAuto();
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
            return Map.of();
        }
    }

    /**
     * Returns true if a valid target is detected.
     * @return true if targetValid is non-zero.
     */
    public boolean hasValidTarget() {
        updateIfAuto();
        return targetValid != 0.0;
    }

    /**
     * Gets the last known X offset.
     * @return target X offset.
     */
    public double getTargetX() {
        updateIfAuto();
        return targetX;
    }

    /**
     * Gets the last known Y offset.
     * @return target Y offset.
     */
    public double getTargetY() {
        updateIfAuto();
        return targetY;
    }

    /**
     * Gets the last known target area.
     * @return target area.
     */
    public double getTargetArea() {
        updateIfAuto();
        return targetArea;
    }

    /**
     * Gets the last known target validity.
     * @return target validity (0 or 1).
     */
    public double getTargetValid() {
        updateIfAuto();
        return targetValid;
    }

    /**
     * Gets the last known target ID.
     * @return target ID.
     */
    public double getTargetID() {
        updateIfAuto();
        return targetID;
    }

    /**
     * Gets the last known Limelight JSON payload.
     * @return JSON string.
     */
    public String getTargetJson() {
        updateIfAuto();
        return targetJSON;
    }

}
