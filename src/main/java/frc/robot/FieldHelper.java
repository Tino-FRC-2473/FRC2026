package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.VisionConstants.TAG_LAYOUT;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SimConstants;

/*
 * TOTAL # OF APRIL TAGS: 32
 * 
 * BLUE HUB APRIL TAGS (8 TOTAL): 25, 26, 27, 18, 19, 20, 21, 24
 * RED HUB APRIL TAGS (8 TOTAL): 9, 10, 11, 2, 3, 4, 5, 8
 * 
 * RIGHT SIDE BLUE TRENCH (2 TOTAL): 28 (INSIDE), 17 (OUTSIDE)
 * LEFT SIDE BLUE TRENCH (2 TOTAL): 23 (INSIDE), 22 (OUTSIDE)
 * RIGHT SIDE RED TRENCH (2 TOTAL): 12 (INSIDE), 1 (OUTSIDE)
 * LEFT SIDE RED TRENCH (2 TOTAL): 7 (INSIDE), 6 (OUTSIDE)
 * 
 * BLUE TOWER: 31 (CENTER), 32 (LEFT)
 * RED TOWER: 15 (CEMTER), 16 (LEFT)
 * 
 * BLUE DEPOT: 30 (LEFT), 29 (RIGHT)
 * RED DEPOT: 14 (LEFT), 13 (RIGHT)
 */

public final /* singleton */ class FieldHelper {

    // Going counter-clockwise to better understand (Front, Right, Back, Left)
    public enum HubTag {
        A, B, C, D, E, F, G, H
    }

    private static Map<HubTag, AprilTag> hubAprilTags = new HashMap<>();

    // Specific tag IDs for every april tag listed in HubTag
    // This is for the blue side, subtract 16 from all tag IDs for red side...
    public static final int TAG_ID_HUB_SIDE_A = 25;
    public static final int TAG_ID_HUB_SIDE_B = 26;
    public static final int TAG_ID_HUB_SIDE_C = 27;
    public static final int TAG_ID_HUB_SIDE_D = 18;
    public static final int TAG_ID_HUB_SIDE_E = 19;
    public static final int TAG_ID_HUB_SIDE_F = 20;
    public static final int TAG_ID_HUB_SIDE_G = 21;
    public static final int TAG_ID_HUB_SIDE_H = 24;

    // Designed for blue team, subtract 16 for red side
    // Left inside is for the tag of the left trench on your alliance side and everything else should be self-explanatory
    public enum TrenchTag {
        LEFTIN, LEFTOUT, RIGHTIN, RIGHTOUT
    }

    private static Map<TrenchTag, AprilTag> trenchAprilTags = new HashMap<>();

    public static final int TAG_ID_TRENCH_LEFT_IN = 7;
    public static final int TAG_ID_TRENCH_LEFT_OUT = 6;
    public static final int TAG_ID_TRENCH_RIGHT_IN = 12;
    public static final int TAG_ID_TRENCH_RIGHT_OUT = 1;

    // Once again, designed for blue team...
    public enum TowerTag {
        CENTER, LEFT
    }

    private static Map<TowerTag, AprilTag> towerAprilTags = new HashMap<>();

    public static final int TAG_ID_TOWER_CENTER = 31;
    public static final int TAG_ID_TOWER_LEFT = 32;

    public enum DepotTag {
        LEFT, RIGHT
    }

    private static Map<DepotTag, AprilTag> depotAprilTags = new HashMap<>();

    public static final int TAG_ID_DEPOT_RIGHT = 29;
    public static final int TAG_ID_DEPOT_LEFT = 30;

    // List of starting positions
    public enum StartingPose {
        R1(RED1_STARTING_POSITION),
        R2(RED2_STARTING_POSITION),
        R3(RED3_STARTING_POSITION),
        B1(BLUE1_STARTING_POSITION),
        B2(BLUE2_STARTING_POSITION),
        B3(BLUE3_STARTING_POSITION);

        private final Pose2d pose;

        StartingPose(Pose2d thePose) {
            pose = thePose;
        }

        /**
         * Gets the pose.
         *
         * @return the pose
         */
        public Pose2d getPose() {
            return pose;
        }
    }

    // Dimensions are 317.7 inches (width) by 651.2 inches (length)
    // Divided 317.7 inches into quarters and put the robots on those lines for starting positions
    // Alliance line is 158.6 inches into the field
    // Backing up the robots by 5 inches from the starting line (can be adjusted later)

    public static final Pose2d RED1_STARTING_POSITION = new Pose2d(
            3.52044,
            2.017395,
            new Rotation2d(Math.PI));

    public static final Pose2d RED2_STARTING_POSITION = new Pose2d(
            3.52044,
            4.03479,
            new Rotation2d(Math.PI));

    public static final Pose2d RED3_STARTING_POSITION = new Pose2d(
            3.52044,
            6.052185,
            new Rotation2d(Math.PI));

    public static final Pose2d BLUE1_STARTING_POSITION = new Pose2d(
            13.02004,
            2.017395,
            new Rotation2d());
    public static final Pose2d BLUE2_STARTING_POSITION = new Pose2d(
            13.02004,
            13.02004,
            new Rotation2d());
    public static final Pose2d BLUE3_STARTING_POSITION = new Pose2d(
            13.02004,
            6.052185,
            new Rotation2d());

    // Adding all the HubTag IDs with their corresponding April Tags to the hubAprilTags map...
    static {
        hubAprilTags.put(
            HubTag.A,
            new AprilTag(TAG_ID_HUB_SIDE_A, TAG_LAYOUT.getTagPose(TAG_ID_HUB_SIDE_A).orElse(null))
        );
        hubAprilTags.put(
            HubTag.B,
            new AprilTag(TAG_ID_HUB_SIDE_B, TAG_LAYOUT.getTagPose(TAG_ID_HUB_SIDE_B).orElse(null))
        );

        hubAprilTags.put(
            HubTag.C,
            new AprilTag(TAG_ID_HUB_SIDE_C, TAG_LAYOUT.getTagPose(TAG_ID_HUB_SIDE_C).orElse(null))
        );

        hubAprilTags.put(
            HubTag.D,
            new AprilTag(TAG_ID_HUB_SIDE_D, TAG_LAYOUT.getTagPose(TAG_ID_HUB_SIDE_D).orElse(null))
        );

        hubAprilTags.put(
            HubTag.E,
            new AprilTag(TAG_ID_HUB_SIDE_E, TAG_LAYOUT.getTagPose(TAG_ID_HUB_SIDE_E).orElse(null))
        );
        hubAprilTags.put(
            HubTag.F,
            new AprilTag(TAG_ID_HUB_SIDE_F, TAG_LAYOUT.getTagPose(TAG_ID_HUB_SIDE_F).orElse(null))
        );

        hubAprilTags.put(
            HubTag.G,
            new AprilTag(TAG_ID_HUB_SIDE_G, TAG_LAYOUT.getTagPose(TAG_ID_HUB_SIDE_G).orElse(null))
        );
        
        hubAprilTags.put(
            HubTag.H,
            new AprilTag(TAG_ID_HUB_SIDE_H, TAG_LAYOUT.getTagPose(TAG_ID_HUB_SIDE_H).orElse(null))
        );
    }

    // Doing the same with trenches...
    static {
        trenchAprilTags.put(
            TrenchTag.LEFTIN,
            new AprilTag(TAG_ID_TRENCH_LEFT_IN, TAG_LAYOUT.getTagPose(TAG_ID_TRENCH_LEFT_IN).orElse(null))
        );
        trenchAprilTags.put(
            TrenchTag.LEFTOUT,
            new AprilTag(TAG_ID_TRENCH_LEFT_OUT, TAG_LAYOUT.getTagPose(TAG_ID_TRENCH_LEFT_OUT).orElse(null))
        );

        trenchAprilTags.put(
            TrenchTag.RIGHTIN,
            new AprilTag(TAG_ID_TRENCH_RIGHT_IN, TAG_LAYOUT.getTagPose(TAG_ID_TRENCH_RIGHT_IN).orElse(null))
        );

        trenchAprilTags.put(
            TrenchTag.RIGHTOUT,
            new AprilTag(TAG_ID_TRENCH_RIGHT_OUT, TAG_LAYOUT.getTagPose(TAG_ID_TRENCH_RIGHT_OUT).orElse(null))
        );
    }

    // And depots + towers...
    static {
        towerAprilTags.put(
            TowerTag.LEFT,
            new AprilTag(TAG_ID_TOWER_LEFT, TAG_LAYOUT.getTagPose(TAG_ID_TOWER_LEFT).orElse(null))
        );
        towerAprilTags.put(
            TowerTag.CENTER,
            new AprilTag(TAG_ID_TOWER_CENTER, TAG_LAYOUT.getTagPose(TAG_ID_TOWER_CENTER).orElse(null))
        );

        depotAprilTags.put(
            DepotTag.RIGHT,
            new AprilTag(TAG_ID_DEPOT_RIGHT, TAG_LAYOUT.getTagPose(TAG_ID_DEPOT_RIGHT).orElse(null))
        );

        depotAprilTags.put(
            DepotTag.LEFT,
            new AprilTag(TAG_ID_DEPOT_LEFT, TAG_LAYOUT.getTagPose(TAG_ID_DEPOT_LEFT).orElse(null))
        );
    }

    /**
     * Get a map of the hub april tags.
     *
     * @return a map of the hub april tags
     */
    public static Map<HubTag, AprilTag> getHubTags() {
        return hubAprilTags;
    }

    /**
     * Get a map of the trench april tags.
     *
     * @return a map of the trench april tags
     */
    public static Map<TrenchTag, AprilTag> getTrenchTags() {
        return trenchAprilTags;
    }

    /**
     * Get a map of the tower april tags.
     *
     * @return a map of the tower april tags
     */
    public static Map<TowerTag, AprilTag> getTowerTags() {
        return towerAprilTags;
    }

    /**
     * Get a map of the depot april tags.
     *
     * @return a map of the depot april tags
     */
    public static Map<DepotTag, AprilTag> getDepotTags() {
        return depotAprilTags;
    }

    /**
     * Get the robot starting position based off alliance and location
     * 
     * @param alliance the alliance you are on
     * @param location location of the robot 
     */
    public static Pose2d getStartingPose(Alliance alliance, int location) {
        if (alliance == Alliance.Red) {
            switch (location) {
                case 1:
                    return StartingPose.R1.getPose();
                case 2:
                    return StartingPose.R2.getPose();
                case 3:
                    return StartingPose.R3.getPose();
                default:
                    throw new IllegalArgumentException("Invalid location: " + location);
            }
        } else {
            switch (location) {
                case 1:
                    return StartingPose.B1.getPose();
                case 2:
                    return StartingPose.B2.getPose();
                case 3:
                    return StartingPose.B3.getPose();
                default:
                    throw new IllegalArgumentException("Invalid location: " + location);
            }
        }
    }

    public static void getAlignedDesiredPoseForHub() {
        //TODO: Add functioning code here
    }

    public static void getAlignedDesiredPoseForTrench() {
        //TODO: Add functioning code here
    }

    public static void getAlignedDesiredPoseForTower() {
        //TODO: Add functioning code here
    }

    public static void getAlignedDesiredPoseForDepot() {
        //TODO: Add functioning code here
    }   
}