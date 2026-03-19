package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.ShooterConstants;

public class LaunchCalculator {
	private static LaunchCalculator instance;

	private final LinearFilter driveAngleFilter =
			LinearFilter.movingAverage((int) (0.1 / 0.02)); // 0.02 is default loop period

	private Rotation2d lastDriveAngle;

	public static LaunchCalculator getInstance() {
		if (instance == null) instance = new LaunchCalculator();
		return instance;
	}

	public record LaunchingParameters(
			boolean isValid,
			Rotation2d driveAngle,
			double targetDriveVelocity,
			double flywheelSpeed,
			double distance,
			double timeOfFlight,
			boolean passing) {}

	// Cache parameters
	private LaunchingParameters latestParameters = null;

	// Phase delay for shooting on the move
	private static final double phaseDelay = 0.03;

	// Launching Maps
	private static final InterpolatingDoubleTreeMap flywheelSpeedMap =
			new InterpolatingDoubleTreeMap();
	private static final InterpolatingDoubleTreeMap timeOfFlightMap =
			new InterpolatingDoubleTreeMap();

	// Passing Maps
	private static final InterpolatingDoubleTreeMap passingFlywheelSpeedMap =
			new InterpolatingDoubleTreeMap();
	private static final InterpolatingDoubleTreeMap passingTimeOfFlightMap =
			new InterpolatingDoubleTreeMap();

	static {
		// Public code maps 150-190 values. 
		// Converting (what is likely) Rads/Sec to RPS (divide by 2PI)
		flywheelSpeedMap.put(0.96, 150.0 / (2 * Math.PI));
		flywheelSpeedMap.put(1.16, 155.0 / (2 * Math.PI));
		flywheelSpeedMap.put(1.58, 160.0 / (2 * Math.PI));
		flywheelSpeedMap.put(2.07, 165.0 / (2 * Math.PI));
		flywheelSpeedMap.put(2.37, 170.0 / (2 * Math.PI));
		flywheelSpeedMap.put(2.47, 170.0 / (2 * Math.PI));
		flywheelSpeedMap.put(2.70, 170.0 / (2 * Math.PI));
		flywheelSpeedMap.put(2.94, 175.0 / (2 * Math.PI));
		flywheelSpeedMap.put(3.48, 175.0 / (2 * Math.PI));
		flywheelSpeedMap.put(3.92, 180.0 / (2 * Math.PI));
		flywheelSpeedMap.put(4.35, 185.0 / (2 * Math.PI));
		flywheelSpeedMap.put(4.84, 190.0 / (2 * Math.PI));

		timeOfFlightMap.put(5.68, 1.16);
		timeOfFlightMap.put(4.55, 1.12);
		timeOfFlightMap.put(3.15, 1.11);
		timeOfFlightMap.put(1.88, 1.09);
		timeOfFlightMap.put(1.38, 0.90);

		passingFlywheelSpeedMap.put(5.46, 160.0 / (2 * Math.PI));
		passingFlywheelSpeedMap.put(6.62, 180.0 / (2 * Math.PI));
		passingFlywheelSpeedMap.put(7.80, 200.0 / (2 * Math.PI));
		passingFlywheelSpeedMap.put(17.16, 360.0 / (2 * Math.PI));

		passingTimeOfFlightMap.put(5.46, 1.27);
		passingTimeOfFlightMap.put(6.62, 1.39);
		passingTimeOfFlightMap.put(7.80, 1.49);
		passingTimeOfFlightMap.put(11.0, 1.75);
		passingTimeOfFlightMap.put(13.0, 1.76);
		passingTimeOfFlightMap.put(17.16, 2.16);
	}

	public LaunchingParameters getParameters(Pose2d robotPose, ChassisSpeeds robotVelocity) {
		return calculateForTarget(robotPose, robotVelocity, DrivetrainConstants.RED_HUB_POSE.getTranslation(), false);
	}

	public LaunchingParameters getParameters(Pose2d robotPose, ChassisSpeeds robotVelocity, Translation2d target, boolean isPassing) {
		return calculateForTarget(robotPose, robotVelocity, target, isPassing);
	}

	private LaunchingParameters calculateForTarget(Pose2d robotPose, ChassisSpeeds robotVelocity, Translation2d target, boolean isPassing) {

		// Calculate estimated pose while accounting for phase delay
		Pose2d estimatedPose = robotPose.exp(
				new Twist2d(
						robotVelocity.vxMetersPerSecond * phaseDelay,
						robotVelocity.vyMetersPerSecond * phaseDelay,
						robotVelocity.omegaRadiansPerSecond * phaseDelay));

		// Define launcher position on the field
		Pose2d launcherPosition = estimatedPose.transformBy(ShooterConstants.ROBOT_TO_LAUNCHER.toTransform2d());
		double launcherToTargetDistance = target.getDistance(launcherPosition.getTranslation());

		// Velocity of the robot in field-relative terms is needed for lookahead.
		// ChassisSpeeds is robot-relative, so we need to translate.
		// If autonomous, we could use setpoint velocity, but here we'll use actual measured chassis speeds projected field-relative:
		double vxField = robotVelocity.vxMetersPerSecond * estimatedPose.getRotation().getCos() - robotVelocity.vyMetersPerSecond * estimatedPose.getRotation().getSin();
		double vyField = robotVelocity.vxMetersPerSecond * estimatedPose.getRotation().getSin() + robotVelocity.vyMetersPerSecond * estimatedPose.getRotation().getCos();
		ChassisSpeeds fieldRelativeVelocity = new ChassisSpeeds(vxField, vyField, robotVelocity.omegaRadiansPerSecond);

		double timeOfFlight = isPassing
				? passingTimeOfFlightMap.get(launcherToTargetDistance)
				: timeOfFlightMap.get(launcherToTargetDistance);
		Pose2d lookaheadPose = launcherPosition;
		double lookaheadLauncherToTargetDistance = launcherToTargetDistance;

		// Iterative lookahead for shooting on the move
		for (int i = 0; i < 20; i++) {
			timeOfFlight = isPassing
					? passingTimeOfFlightMap.get(lookaheadLauncherToTargetDistance)
					: timeOfFlightMap.get(lookaheadLauncherToTargetDistance);
			double offsetX = fieldRelativeVelocity.vxMetersPerSecond * timeOfFlight;
			double offsetY = fieldRelativeVelocity.vyMetersPerSecond * timeOfFlight;
			lookaheadPose =
					new Pose2d(
							launcherPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
							launcherPosition.getRotation());
			lookaheadLauncherToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
		}

		// Account for launcher being off center
		Pose2d lookaheadRobotPose = lookaheadPose.transformBy(ShooterConstants.ROBOT_TO_LAUNCHER.toTransform2d().inverse());
		Rotation2d driveAngle = getDriveAngleWithLauncherOffset(lookaheadRobotPose, target);

		if (lastDriveAngle == null) lastDriveAngle = driveAngle;
		double driveVelocitySetpoint = driveAngleFilter.calculate(driveAngle.minus(lastDriveAngle).getRadians() / 0.02);
		lastDriveAngle = driveAngle;

		double flywheelVelocity = isPassing
				? passingFlywheelSpeedMap.get(lookaheadLauncherToTargetDistance)
				: flywheelSpeedMap.get(lookaheadLauncherToTargetDistance);

		boolean isValid = lookaheadLauncherToTargetDistance >= 0.9 && lookaheadLauncherToTargetDistance <= 17.16;

		latestParameters = new LaunchingParameters(
				isValid,
				driveAngle,
				driveVelocitySetpoint,
				flywheelVelocity,
				lookaheadLauncherToTargetDistance,
				timeOfFlight,
				isPassing);

		return latestParameters;
	}

	private static Rotation2d getDriveAngleWithLauncherOffset(Pose2d robotPose, Translation2d target) {
		Rotation2d fieldToHubAngle = target.minus(robotPose.getTranslation()).getAngle();
		Rotation2d hubAngle =
				new Rotation2d(
						Math.asin(
								MathUtil.clamp(
										ShooterConstants.ROBOT_TO_LAUNCHER.getY()
												/ target.getDistance(robotPose.getTranslation()),
										-1.0,
										1.0)));
		Rotation2d driveAngle = fieldToHubAngle.plus(hubAngle).plus(ShooterConstants.ROBOT_TO_LAUNCHER.getRotation().toRotation2d());
		return driveAngle;
	}

	public void clearLaunchingParameters() {
		latestParameters = null;
	}
}
