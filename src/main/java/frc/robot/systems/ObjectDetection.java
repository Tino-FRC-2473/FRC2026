package frc.robot;
public class ObjectDetection {
	// method for finding fuel
public static void main(String[] args){
public static LimelightHelpers.RawDetection[] getRawDetections(String limelightName){
RawDetection[] results =  LimelightHelpers.getRawDetections(limelightName);
return results;

    }
    // Gets the average color under the crosshair region as a 3-element array.
    public static double[] getTargetColor(String limelightName){
        double[] tColor = LimelightHelpers.getTargetColor(limelightName);
        return tColor;
    }
    // returns Target area percentage (0-100)
    public static double getTA(String limelightName){
    return LimelightHelpers.getTA(limelightName);
    }
    // Gets the number of targets or fuels currently detected.
    public static int getTargetCount(String limelightName){
     return LimelightHelpers.getTargetCount(limelightName);
    }
    
// Gets the latest JSON results output and returns a LimelightResults object.
public static LimelightHelpers.LimelightResults getLatestResults(String limelightName){
return LimelightHelpers.getLatestResults( limelightName);
}
   }

}