package frc.robot.systems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    private final NetworkTable limelightTable;

    // Setting limelightTable to the actual table since it's too much typing if
    // I have to type 2 different methods everytime I want to get a value
    public Limelight() {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    // Does camera see a target? (Note to self: set pipelines for fuel)
    public boolean hasTarget() {
        return limelightTable.getEntry("tv").getDouble(0) == 1;
    }

    // Horizontal offset from target
    public double getXOffset() {
        return limelightTable.getEntry("tx").getDouble(0);
    }

    // Vertical offset from target
    public double getYOffset() {
        return limelightTable.getEntry("ty").getDouble(0);
    }

    // Target area (size) (Big number = more space taken up on camera screen and v.v.)
    public double getArea() {
        return limelightTable.getEntry("ta").getDouble(0);
    }
}
