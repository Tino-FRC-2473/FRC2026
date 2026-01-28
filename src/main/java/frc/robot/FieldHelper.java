package frc.robot;

import static frc.robot.Constants.VisionConstants.TAG_LAYOUT;
import java.util.HashMap;
import java.util.Map;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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
* RED TOWER: 15 (CENTER), 16 (LEFT)
*
* BLUE DEPOT: 30 (LEFT), 29 (RIGHT)
* RED DEPOT: 14 (LEFT), 13 (RIGHT)
*/

public final /* singleton */ class FieldHelper {

	// Going counter-clockwise to better understand (Front, Right, Back, Left)
	// Specific tag IDs for every april tag listed in HubTag
	// This is for the blue side, subtract 16 from all tag IDs for red side...

	public enum HubTag {
		A, B, C, D, E, F, G, H
	}

	public enum HubSide {
		NEUTRAL, ALLIANCE
	}

	public static final int TAGIDHUBSIDEA = 25;
	public static final int TAGIDHUBSIDEB = 26;
	public static final int TAGIDHUBSIDEC = 27;
	public static final int TAGIDHUBSIDED = 18;
	public static final int TAGIDHUBSIDEE = 19;
	public static final int TAGIDHUBSIDEF = 20;
	public static final int TAGIDHUBSIDEG = 21;
	public static final int TAGIDHUBSIDEH = 24;

	// Doing the same for towers...
	// Designed for blue team, subtract 16 for red side
	// Once again, designed for blue team...
	public enum TowerTag {
		CENTER, LEFT
	}

	public enum TowerSide {
		CENTER, LEFT, RIGHT
	}

	public static final int TAGIDTOWERCENTER = 31;
	public static final int TAGIDTOWERLEFT = 32;

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
		4.03479,
		new Rotation2d());
	public static final Pose2d BLUE3_STARTING_POSITION = new Pose2d(
		13.02004,
		6.052185,
		new Rotation2d());

	private static HashMap<HubTag, AprilTag> hubAprilTags;
	private static HashMap<TowerTag, AprilTag> towerAprilTags;
	// Adding all the HubTag IDs with their corresponding April Tags to the hubAprilTags map...
	{
		hubAprilTags = new HashMap<>(
			Map.of(
				HubTag.A,
				new AprilTag(
					TAGIDHUBSIDEA,
					TAG_LAYOUT.getTagPose(TAGIDHUBSIDEA).orElseThrow()
				),
				HubTag.B,
				new AprilTag(
					TAGIDHUBSIDEB,
					TAG_LAYOUT.getTagPose(TAGIDHUBSIDEB).orElseThrow(null)
				),
				HubTag.C,
				new AprilTag(
					TAGIDHUBSIDEC,
					TAG_LAYOUT.getTagPose(TAGIDHUBSIDEC).orElseThrow(null)
				),
				HubTag.D,
				new AprilTag(
					TAGIDHUBSIDED,
					TAG_LAYOUT.getTagPose(TAGIDHUBSIDED).orElseThrow(null)
				),
				HubTag.E,
				new AprilTag(
					TAGIDHUBSIDEE,
					TAG_LAYOUT.getTagPose(TAGIDHUBSIDEE).orElseThrow(null)
				),
				HubTag.F,
				new AprilTag(
					TAGIDHUBSIDEF,
					TAG_LAYOUT.getTagPose(TAGIDHUBSIDEF).orElseThrow(null)
				),
				HubTag.G,
				new AprilTag(
					TAGIDHUBSIDEG,
					TAG_LAYOUT.getTagPose(TAGIDHUBSIDEG).orElseThrow(null)
				),
				HubTag.H,
				new AprilTag(
					TAGIDHUBSIDEH,
					TAG_LAYOUT.getTagPose(TAGIDHUBSIDEH).orElseThrow(null)
				)

			)
		);

		towerAprilTags = new HashMap<>(
			Map.of(
				TowerTag.LEFT,
				new AprilTag(
					TAGIDTOWERLEFT,
					TAG_LAYOUT.getTagPose(TAGIDTOWERLEFT).orElseThrow(null)
				),
				TowerTag.CENTER,
				new AprilTag(
					TAGIDTOWERCENTER,
					TAG_LAYOUT.getTagPose(TAGIDTOWERCENTER).orElseThrow(null)
				)
			)
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
	 * Get a map of the tower april tags.
	 *
	 * @return a map of the tower april tags
	 */
	public static Map<TowerTag, AprilTag> getTowerTags() {
		return towerAprilTags;
	}

	private static final int LOCATION_1 = 1;
	private static final int LOCATION_2 = 2;
	private static final int LOCATION_3 = 3;

	/** Get the robot starting position based off alliance and location.
	 *
	 * @param alliance the alliance you are on
	 * @param location starting location of the robot
	 * @return Pose2d with the location of the robot
	 */
	public static Pose2d getStartingPose(Alliance alliance, int location) {
		if (alliance == Alliance.Red) {
			switch (location) {
				case LOCATION_1:
					return StartingPose.R1.getPose();
				case LOCATION_2:
					return StartingPose.R2.getPose();
				case LOCATION_3:
					return StartingPose.R3.getPose();
				default:
					throw new IllegalArgumentException("Invalid location: " + location);
			}
		} else {
			switch (location) {
				case LOCATION_1:
					return StartingPose.B1.getPose();
				case LOCATION_2:
					return StartingPose.B2.getPose();
				case LOCATION_3:
					return StartingPose.B3.getPose();
				default:
					throw new IllegalArgumentException("Invalid location: " + location);
			}
		}
	}

	/**
	 * Gets the desired pose to align to for the hub.
	 * @param hubSide the side of the hub that the robot is on
	 */
	public static void getAlignedDesiredPoseForHub(HubSide hubSide) {
		//TODO: Add functioning code here
		throw new UnsupportedOperationException();
	}

	/**
	 * Gets the desired pose to align to for the tower.
	 * @param towerSide the side of the tower that the robot is on
	 */
	public static void getAlignedDesiredPoseForTower(TowerSide towerSide) {
		//TODO: Add functioning code here
		throw new UnsupportedOperationException();
	}
}
