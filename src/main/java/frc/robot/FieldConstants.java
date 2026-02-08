package frc.robot;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Path;

/**
 * Contains information for location of field element and other useful reference points.
 *
 * <p>NOTE: All constants are defined relative to the field coordinate system, and from the
 * perspective of the blue alliance station
 */
public class FieldConstants {
	public static final FieldType FIELDTYPE = FieldType.WELDED;
	public static final AprilTagLayoutType DEFAULTAPRILTAGTYPE = AprilTagLayoutType.OFFICIAL;

	// AprilTag related constants
	public static final int APRILTAGCOUNT =
		DEFAULTAPRILTAGTYPE.getLayout().getTags().size();
	public static final double APRILTAGWIDTH = Units.inchesToMeters(6.5);

	// Field dimensions
	public static final double FIELDLENGTH =
		DEFAULTAPRILTAGTYPE.getLayout().getFieldLength();
	public static final double FIELDWIDTH = DEFAULTAPRILTAGTYPE.getLayout().getFieldWidth();

	/**
	 * Officially defined and relevant vertical lines found on the field (defined by X-axis offset).
	 */
	public static class LinesVertical {
		public static final double CENTER = FIELDLENGTH / 2.0;
		public static final double STARTING =
				DEFAULTAPRILTAGTYPE.getLayout().getTagPose(26).get().getX();
		public static final double ALLIANCEZONE = STARTING;
		public static final double HUBCENTER =
				DEFAULTAPRILTAGTYPE.getLayout().getTagPose(26).get().getX() + Hub.WIDTH / 2.0;
		public static final double NEUTRALZONENEAR = CENTER - Units.inchesToMeters(120);
		public static final double NEUTRALZONEFAR = CENTER + Units.inchesToMeters(120);
		public static final double OPPHUBCENTER =
				DEFAULTAPRILTAGTYPE.getLayout().getTagPose(4).get().getX() + Hub.WIDTH / 2.0;
		public static final double OPPALLIANCEZONE =
				DEFAULTAPRILTAGTYPE.getLayout().getTagPose(10).get().getX();
	}

	/**
	 * Officially defined/relevant horizontal lines found on the field (defined by Y-axis offset).
	 *
	 * <p>NOTE: The field element start and end are always left to right from the perspective of the
	 * alliance station
	 */
	public static class LinesHorizontal {

		public static final double CENTER = FIELDWIDTH / 2.0;

		// Right of hub
		public static final double RIGHTBUMPSTART = Hub.NEARRIGHTCORNER.getY();
		public static final double RIGHTBUMPEND = RIGHTBUMPSTART - RightBump.WIDTH;
		public static final double RIGHTTRENCHOPENSTART = RIGHTBUMPEND - Units.inchesToMeters(12.0);
		public static final double RIGHTTRENCHOPENEND = 0;

		// Left of hub
		public static final double LEFTBUMPEND = Hub.NEARLEFTCORNER.getY();
		public static final double LEFTBUMPSTART = LEFTBUMPEND + LeftBump.WIDTH;
		public static final double LEFTTRENCHOPENEND = LEFTBUMPSTART + Units.inchesToMeters(12.0);
		public static final double LEFTTRENCHOPENSTART = FIELDWIDTH;
	}

	/** Hub related constants. */
	public static class Hub {

		// Dimensions
		public static final double WIDTH = Units.inchesToMeters(47.0);
		public static final double HEIGHT =
				Units.inchesToMeters(72.0); // includes the catcher at the top
		public static final double INNERWIDTH = Units.inchesToMeters(41.7);
		public static final double INNERHEIGHT = Units.inchesToMeters(56.5);

		// Relevant reference points on alliance side
		public static final Translation3d TOPCENTERPOINT =
				new Translation3d(
						DEFAULTAPRILTAGTYPE.getLayout().getTagPose(26).get().getX() + WIDTH / 2.0,
						FIELDWIDTH / 2.0,
						HEIGHT);
		public static final Translation3d INNERCENTERPOINT =
				new Translation3d(
						DEFAULTAPRILTAGTYPE.getLayout().getTagPose(26).get().getX() + WIDTH / 2.0,
						FIELDWIDTH / 2.0,
						INNERHEIGHT);

		public static final Translation2d NEARLEFTCORNER =
				new Translation2d(
					TOPCENTERPOINT.getX() - WIDTH / 2.0, FIELDWIDTH / 2.0 + WIDTH / 2.0
				);
		public static final Translation2d NEARRIGHTCORNER =
				new Translation2d(
					TOPCENTERPOINT.getX() - WIDTH / 2.0, FIELDWIDTH / 2.0 - WIDTH / 2.0
				);
		public static final Translation2d FARLEFTCORNER =
				new Translation2d(
					TOPCENTERPOINT.getX() + WIDTH / 2.0, FIELDWIDTH / 2.0 + WIDTH / 2.0
				);
		public static final Translation2d FARRIGHTCORNER =
				new Translation2d(
					TOPCENTERPOINT.getX() + WIDTH / 2.0, FIELDWIDTH / 2.0 - WIDTH / 2.0
				);

		// Relevant reference points on the opposite side
		public static final Translation3d OPPTOPCENTERPOINT =
				new Translation3d(
						DEFAULTAPRILTAGTYPE.getLayout().getTagPose(4).get().getX() + WIDTH / 2.0,
						FIELDWIDTH / 2.0,
						HEIGHT);
		public static final Translation2d OPPNEARLEFTCORNER =
				new Translation2d(
					OPPTOPCENTERPOINT.getX() - WIDTH / 2.0, FIELDWIDTH / 2.0 + WIDTH / 2.0
				);
		public static final Translation2d OPPNEARRIGHTCORNER =
				new Translation2d(
					OPPTOPCENTERPOINT.getX() - WIDTH / 2.0, FIELDWIDTH / 2.0 - WIDTH / 2.0
				);
		public static final Translation2d OPPFARLEFTCORNER =
				new Translation2d(
					OPPTOPCENTERPOINT.getX() + WIDTH / 2.0, FIELDWIDTH / 2.0 + WIDTH / 2.0
				);
		public static final Translation2d OPPFARRIGHTCORNER =
				new Translation2d(
					OPPTOPCENTERPOINT.getX() + WIDTH / 2.0, FIELDWIDTH / 2.0 - WIDTH / 2.0
				);

		// Hub faces
		public static final Pose2d NEARFACE =
				DEFAULTAPRILTAGTYPE.getLayout().getTagPose(26).get().toPose2d();
		public static final Pose2d FARFACE =
				DEFAULTAPRILTAGTYPE.getLayout().getTagPose(20).get().toPose2d();
		public static final Pose2d RIGHTFACE =
				DEFAULTAPRILTAGTYPE.getLayout().getTagPose(18).get().toPose2d();
		public static final Pose2d LEFTFACE =
				DEFAULTAPRILTAGTYPE.getLayout().getTagPose(21).get().toPose2d();
	}

	/** Left Bump related constants. */
	public static class LeftBump {

		// Dimensions
		public static final double WIDTH = Units.inchesToMeters(73.0);
		public static final double HEIGHT = Units.inchesToMeters(6.513);
		public static final double DEPTH = Units.inchesToMeters(44.4);

		// Relevant reference points on alliance side
		public static final Translation2d NEARLEFTCORNER =
				new Translation2d(LinesVertical.HUBCENTER - WIDTH / 2, Units.inchesToMeters(255));
		public static final Translation2d NEARRIGHTCORNER = Hub.NEARLEFTCORNER;
		public static final Translation2d FARLEFTCORNER =
				new Translation2d(LinesVertical.HUBCENTER + WIDTH / 2, Units.inchesToMeters(255));
		public static final Translation2d FARRIGHTCORNER = Hub.FARLEFTCORNER;

		// Relevant reference points on opposing side
		public static final Translation2d OPPNEARLEFTCORNER =
				new Translation2d(LinesVertical.HUBCENTER - WIDTH / 2, Units.inchesToMeters(255));
		public static final Translation2d OPPNEARRIGHTCORNER = Hub.OPPNEARLEFTCORNER;
		public static final Translation2d OPPFARLEFTCORNER =
				new Translation2d(LinesVertical.HUBCENTER + WIDTH / 2, Units.inchesToMeters(255));
		public static final Translation2d OPPFARRIGHTCORNER = Hub.OPPFARLEFTCORNER;
	}

	/** Right Bump related constant. */
	public static class RightBump {
		// Dimensions
		public static final double WIDTH = Units.inchesToMeters(73.0);
		public static final double HEIGHT = Units.inchesToMeters(6.513);
		public static final double DEPTH = Units.inchesToMeters(44.4);

		// Relevant reference points on alliance side
		public static final Translation2d NEARLEFTCORNER =
				new Translation2d(LinesVertical.HUBCENTER + WIDTH / 2, Units.inchesToMeters(255));
		public static final Translation2d NEARRIGHTCORNER = Hub.NEARLEFTCORNER;
		public static final Translation2d FARLEFTCORNER =
				new Translation2d(LinesVertical.HUBCENTER - WIDTH / 2, Units.inchesToMeters(255));
		public static final Translation2d FARRIGHTCORNER = Hub.FARLEFTCORNER;

		// Relevant reference points on opposing side
		public static final Translation2d OPPNEARLEFTCORNER =
				new Translation2d(LinesVertical.HUBCENTER + WIDTH / 2, Units.inchesToMeters(255));
		public static final Translation2d OPPNEARRIGHTCORNER = Hub.OPPNEARLEFTCORNER;
		public static final Translation2d OPPFARLEFTCORNER =
				new Translation2d(LinesVertical.HUBCENTER - WIDTH / 2, Units.inchesToMeters(255));
		public static final Translation2d OPPFARRIGHTCORNER = Hub.OPPFARLEFTCORNER;
	}

	/** Left Trench related constants. */
	public static class LeftTrench {
		// Dimensions
		public static final double WIDTH = Units.inchesToMeters(65.65);
		public static final double DEPTH = Units.inchesToMeters(47.0);
		public static final double HEIGHT = Units.inchesToMeters(40.25);
		public static final double OPENINGWIDTH = Units.inchesToMeters(50.34);
		public static final double OPENINGHEIGHT = Units.inchesToMeters(22.25);

		// Relevant reference points on alliance side
		public static final Translation3d OPENINGTOPLEFT =
				new Translation3d(LinesVertical.HUBCENTER, FIELDWIDTH, OPENINGHEIGHT);
		public static final Translation3d OPENINGTOPRIGHT =
				new Translation3d(
					LinesVertical.HUBCENTER, FIELDWIDTH - OPENINGWIDTH, OPENINGHEIGHT
				);

		// Relevant reference points on opposing side
		public static final Translation3d OPPOPENINGTOPLEFT =
				new Translation3d(LinesVertical.OPPHUBCENTER, FIELDWIDTH, OPENINGHEIGHT);
		public static final Translation3d OPPOPENINGTOPRIGHT =
				new Translation3d(
					LinesVertical.OPPHUBCENTER, FIELDWIDTH - OPENINGWIDTH, OPENINGHEIGHT
				);
	}

	public static class RightTrench {

		// Dimensions
		public static final double WIDTH = Units.inchesToMeters(65.65);
		public static final double DEPTH = Units.inchesToMeters(47.0);
		public static final double HEIGHT = Units.inchesToMeters(40.25);
		public static final double OPENINGWIDTH = Units.inchesToMeters(50.34);
		public static final double OPENINGHEIGHT = Units.inchesToMeters(22.25);

		// Relevant reference points on alliance side
		public static final Translation3d OPENINGTOPLEFT =
				new Translation3d(LinesVertical.HUBCENTER, OPENINGWIDTH, OPENINGHEIGHT);
		public static final Translation3d OPENINGTOPRIGHT =
				new Translation3d(LinesVertical.HUBCENTER, 0, OPENINGHEIGHT);

		// Relevant reference points on opposing side
		public static final Translation3d OPPOPENINGTOPLEFT =
				new Translation3d(LinesVertical.OPPHUBCENTER, OPENINGWIDTH, OPENINGHEIGHT);
		public static final Translation3d OPPOPENINGTOPRIGHT =
				new Translation3d(LinesVertical.OPPHUBCENTER, 0, OPENINGHEIGHT);
	}

	/** Tower related constants. */
	public static class Tower {
		// Dimensions
		public static final double WIDTH = Units.inchesToMeters(49.25);
		public static final double DEPTH = Units.inchesToMeters(45.0);
		public static final double HEIGHT = Units.inchesToMeters(78.25);
		public static final double INNEROPENINGWIDTH = Units.inchesToMeters(32.250);
		public static final double FRONTFACEX = Units.inchesToMeters(43.51);

		public static final double UPRIGHTHEIGHT = Units.inchesToMeters(72.1);

		// Rung HEIGHTs from the floor
		public static final double LOWRUNGHEIGHT = Units.inchesToMeters(27.0);
		public static final double MIDRUNGHEIGHT = Units.inchesToMeters(45.0);
		public static final double HIGHRUNGHEIGHT = Units.inchesToMeters(63.0);

		// Relevant reference points on alliance side
		public static final Translation2d CENTERPOINT =
				new Translation2d(
						FRONTFACEX, DEFAULTAPRILTAGTYPE.getLayout().getTagPose(31).get().getY());
		public static final Translation2d LEFTUPRIGHT =
				new Translation2d(
						FRONTFACEX,
						(DEFAULTAPRILTAGTYPE.getLayout().getTagPose(31).get().getY())
								+ INNEROPENINGWIDTH / 2
								+ Units.inchesToMeters(0.75));
		public static final Translation2d RIGHTUPRIGHT =
				new Translation2d(
						FRONTFACEX,
						(DEFAULTAPRILTAGTYPE.getLayout().getTagPose(31).get().getY())
								- INNEROPENINGWIDTH / 2
								- Units.inchesToMeters(0.75));

		// Relevant reference points on opposing side
		public static final Translation2d OPPCENTERPOINT =
				new Translation2d(
						FIELDLENGTH - FRONTFACEX,
						DEFAULTAPRILTAGTYPE.getLayout().getTagPose(15).get().getY());
		public static final Translation2d OPPLEFTUPRIGHT =
				new Translation2d(
						FIELDLENGTH - FRONTFACEX,
						(DEFAULTAPRILTAGTYPE.getLayout().getTagPose(15).get().getY())
								+ INNEROPENINGWIDTH / 2
								+ Units.inchesToMeters(0.75));
		public static final Translation2d OPPRIGHTUPRIGHT =
				new Translation2d(
						FIELDLENGTH - FRONTFACEX,
						(DEFAULTAPRILTAGTYPE.getLayout().getTagPose(15).get().getY())
								- INNEROPENINGWIDTH / 2
								- Units.inchesToMeters(0.75));
	}

	public static class Depot {
		// Dimensions
		public static final double WIDTH = Units.inchesToMeters(42.0);
		public static final double DEPTH = Units.inchesToMeters(27.0);
		public static final double HEIGHT = Units.inchesToMeters(1.125);
		public static final double DISTANCEFROMCENTERY = Units.inchesToMeters(75.93);

		// Relevant reference points on alliance side
		public static final Translation3d DEPOTCENTER =
				new Translation3d(DEPTH, (FIELDWIDTH / 2) + DISTANCEFROMCENTERY, HEIGHT);
		public static final Translation3d LEFTCORNER =
				new Translation3d(
					DEPTH, (FIELDWIDTH / 2) + DISTANCEFROMCENTERY + (WIDTH / 2), HEIGHT
				);
		public static final Translation3d RIGHTCORNER =
				new Translation3d(
					DEPTH, (FIELDWIDTH / 2) + DISTANCEFROMCENTERY - (WIDTH / 2), HEIGHT
				);
	}

	public static class Outpost {
		// Dimensions
		public static final double WIDTH = Units.inchesToMeters(31.8);
		public static final double OPENINGDISTANCEFROMFLOOR = Units.inchesToMeters(28.1);
		public static final double HEIGHT = Units.inchesToMeters(7.0);

		// Relevant reference points on alliance side
		public static final Translation2d CENTERPOINT =
				new Translation2d(0, DEFAULTAPRILTAGTYPE.getLayout().getTagPose(29).get().getY());
	}

	public static class StartingPositions {

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
	}

	public enum FieldType {
		ANDYMARK("andymark"),
		WELDED("welded");

		private final String jsonFolder;

		FieldType(String inputJsonFolder) {
			this.jsonFolder = inputJsonFolder;
		}

		/** Returns jsonFolder.
		 * @return string of jsonFolder
		 */
		public String getJsonFolder() {
			return this.jsonFolder;
		}
	}

	public enum AprilTagLayoutType {
		OFFICIAL("2026-official"),
		NONE("2026-none");

		private final String name;
		private volatile AprilTagFieldLayout layout;
		private volatile String layoutString;

		AprilTagLayoutType(String newName) {
			this.name = newName;
		}

		/** Returns the AprilTagFieldLayout.
		 * @return AprilTagFieldLayout
		 */
		public AprilTagFieldLayout getLayout() {
			if (layout == null) {
				synchronized (this) {
					if (layout == null) {
						try {
							Path p =
									Constants.DrivetrainConstants.DISABLEHAL
											? Path.of(
													"src",
													"main",
													"deploy",
													"apriltags",
													FIELDTYPE.getJsonFolder(),
													name + ".json")
											: Path.of(
													Filesystem.getDeployDirectory().getPath(),
													"apriltags",
													FIELDTYPE.getJsonFolder(),
													name + ".json");
							layout = new AprilTagFieldLayout(p);
							layoutString = new ObjectMapper().writeValueAsString(layout);
						} catch (IOException e) {
							throw new RuntimeException(e);
						}
					}
				}
			}
			return layout;
		}

		/** Returns layoutString.
		 * @return layoutString
		 */
		public String getLayoutString() {
			if (layoutString == null) {
				getLayout();
			}
			return layoutString;
		}
	}
}
