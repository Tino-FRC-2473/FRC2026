package frc.robot.systems;
import  limelight.Limelight;
import limelight.networktables.LimelightResults;
import limelight.networktables.target.pipeline.NeuralClassifier;
public class ObjectDetection {
    Limelight limelight = new Limelight("limelight");
// Get the results
limelight.getLatestResults().ifPresent((LimelightResults result) -> {
    for (NeuralClassifier object : result.targets_Classifier)
    {
        // Classifier says its a coral.
        if (object.className.equals("coral"))
        {
            // Check pixel location of coral.
            if (object.ty > 2 && object.ty < 1)
            {
            // Coral is valid! do stuff!
            }
        }
    }
});
}
