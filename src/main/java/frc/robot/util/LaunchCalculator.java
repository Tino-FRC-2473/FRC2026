package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.ShooterConstants;

public class LaunchCalculator {
	private static LaunchCalculator instance;

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
	// Aerodynamic drag coefficient for SOTM decay
	private static final double sotmDragCoeff = 0.47;

	// Warm start state for the Newton solver
	private double previousTOF = -1.0;

	private static final Transform2d ROBOT_TO_LAUNCHER_2D = new Transform2d(
			ShooterConstants.ROBOT_TO_LAUNCHER.getTranslation().toTranslation2d(),
			ShooterConstants.ROBOT_TO_LAUNCHER.getRotation().toRotation2d());

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

	private double getEffectiveTOF(double distance, boolean isPassing) {
		return isPassing ? passingTimeOfFlightMap.get(distance) : timeOfFlightMap.get(distance);
	}

	private double tofMapDerivative(double distance, boolean isPassing) {
		double h = 0.01;
		double tHigh = getEffectiveTOF(distance + h, isPassing);
		double tLow = getEffectiveTOF(distance - h, isPassing);
		return (tHigh - tLow) / (2.0 * h);
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
		Pose2d launcherPosition = estimatedPose.transformBy(ROBOT_TO_LAUNCHER_2D);
		double rx = target.getX() - launcherPosition.getX();
		double ry = target.getY() - launcherPosition.getY();
		double launcherToTargetDistance = Math.hypot(rx, ry);

		// Velocity of the robot in field-relative terms is needed for lookahead.
		// ChassisSpeeds is robot-relative, so we need to translate.
		double vxField = robotVelocity.vxMetersPerSecond * estimatedPose.getRotation().getCos() - robotVelocity.vyMetersPerSecond * estimatedPose.getRotation().getSin();
		double vyField = robotVelocity.vxMetersPerSecond * estimatedPose.getRotation().getSin() + robotVelocity.vyMetersPerSecond * estimatedPose.getRotation().getCos();

		double solvedTOF;
		double lookaheadLauncherToTargetDistance = launcherToTargetDistance;
		Pose2d lookaheadPose = launcherPosition;

		double robotSpeed = Math.hypot(vxField, vyField);

		if (robotSpeed < 0.1) {
			// Static shot
			solvedTOF = getEffectiveTOF(launcherToTargetDistance, isPassing);
			previousTOF = -1.0;
		} else {
			// Newton-method SOTM solver with drag compensation
			double tof = (previousTOF > 0) ? previousTOF : getEffectiveTOF(launcherToTargetDistance, isPassing);
			
			for (int i = 0; i < 5; i++) {
				double prevTOF = tof;
				
				double dragExp = Math.exp(-sotmDragCoeff * tof);
				double driftTOF = (1.0 - dragExp) / sotmDragCoeff;

				// Projected target displacement from launcher
				double prx = rx - vxField * driftTOF;
				double pry = ry - vyField * driftTOF;
				lookaheadLauncherToTargetDistance = Math.hypot(prx, pry);

				if (lookaheadLauncherToTargetDistance < 0.01) {
					tof = getEffectiveTOF(launcherToTargetDistance, isPassing);
					break; // Diverged
				}

				double lookupTOF = getEffectiveTOF(lookaheadLauncherToTargetDistance, isPassing);

				double dPrime = -dragExp * (prx * vxField + pry * vyField) / lookaheadLauncherToTargetDistance;
				double gPrime = tofMapDerivative(lookaheadLauncherToTargetDistance, isPassing);
				double f = lookupTOF - tof;
				double fPrime = gPrime * dPrime - 1.0;

				if (Math.abs(fPrime) > 0.01) {
					tof = tof - f / fPrime;
				} else {
					tof = lookupTOF;
				}

				tof = MathUtil.clamp(tof, 0.05, 5.0);

				if (Math.abs(tof - prevTOF) < 0.001) {
					break;
				}
			}

			// Divergence guard
			if (tof > 5.0 || tof < 0.0 || Double.isNaN(tof)) {
				tof = getEffectiveTOF(launcherToTargetDistance, isPassing);
			}

			solvedTOF = tof;
			previousTOF = solvedTOF;

			// Final lookahead pose calculation for drive angle extraction
			double driftTOF = (1.0 - Math.exp(-sotmDragCoeff * solvedTOF)) / sotmDragCoeff;
			double offsetX = vxField * driftTOF;
			double offsetY = vyField * driftTOF;
			lookaheadPose =
					new Pose2d(
							launcherPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
							launcherPosition.getRotation());
		}

		// Account for launcher being off center
		Pose2d lookaheadRobotPose = lookaheadPose.transformBy(ROBOT_TO_LAUNCHER_2D.inverse());
		Rotation2d driveAngle = getDriveAngleWithLauncherOffset(lookaheadRobotPose, target);

		// Analytical angular velocity feedforward
		double driveVelocitySetpoint = 0.0;
		if (robotSpeed >= 0.1 && lookaheadLauncherToTargetDistance > 0.1) {
			double tangentialVel = (ry * vxField - rx * vyField) / lookaheadLauncherToTargetDistance;
			driveVelocitySetpoint = tangentialVel / lookaheadLauncherToTargetDistance;
		}

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
				solvedTOF,
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
		previousTOF = -1.0;
	}
}
