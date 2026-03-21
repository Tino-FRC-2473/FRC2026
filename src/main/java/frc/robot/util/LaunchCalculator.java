package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.firecontrol.ShotCalculator;
import frc.robot.util.firecontrol.ShotCalculator.LaunchParameters;
import frc.robot.util.firecontrol.ShotCalculator.ShotInputs;

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
			boolean passing,
			double confidence) {}

	private final ShotCalculator hubCalculator;
	private final ShotCalculator passingCalculator;

	private LaunchCalculator() {
		ShotCalculator.Config hubConfig = new ShotCalculator.Config();
		hubConfig.launcherOffsetX = ShooterConstants.ROBOT_TO_LAUNCHER.getX();
		hubConfig.launcherOffsetY = ShooterConstants.ROBOT_TO_LAUNCHER.getY();
		hubConfig.minScoringDistance = 0.5;
		hubConfig.maxScoringDistance = 6.0;
		hubCalculator = new ShotCalculator(hubConfig);

		ShotCalculator.Config passingConfig = new ShotCalculator.Config();
		passingConfig.launcherOffsetX = ShooterConstants.ROBOT_TO_LAUNCHER.getX();
		passingConfig.launcherOffsetY = ShooterConstants.ROBOT_TO_LAUNCHER.getY();
		passingConfig.minScoringDistance = 0.5;
		passingConfig.maxScoringDistance = 20.0;
		passingCalculator = new ShotCalculator(passingConfig);

		// Load Hub LUT
		hubCalculator.loadLUTEntry(0.96, 150.0 / (2 * Math.PI), 1.16);
		hubCalculator.loadLUTEntry(1.16, 155.0 / (2 * Math.PI), 1.12);
		hubCalculator.loadLUTEntry(1.58, 160.0 / (2 * Math.PI), 1.11);
		hubCalculator.loadLUTEntry(2.07, 165.0 / (2 * Math.PI), 1.09);
		hubCalculator.loadLUTEntry(2.37, 170.0 / (2 * Math.PI), 0.90);
		hubCalculator.loadLUTEntry(2.47, 170.0 / (2 * Math.PI), 0.90);
		hubCalculator.loadLUTEntry(2.70, 170.0 / (2 * Math.PI), 0.90);
		hubCalculator.loadLUTEntry(2.94, 175.0 / (2 * Math.PI), 0.90);
		hubCalculator.loadLUTEntry(3.48, 175.0 / (2 * Math.PI), 0.90);
		hubCalculator.loadLUTEntry(3.92, 180.0 / (2 * Math.PI), 0.90);
		hubCalculator.loadLUTEntry(4.35, 185.0 / (2 * Math.PI), 0.90);
		hubCalculator.loadLUTEntry(4.84, 190.0 / (2 * Math.PI), 0.90);

		// Load Passing LUT
		passingCalculator.loadLUTEntry(5.46, 160.0 / (2 * Math.PI), 1.27);
		passingCalculator.loadLUTEntry(6.62, 180.0 / (2 * Math.PI), 1.39);
		passingCalculator.loadLUTEntry(7.80, 200.0 / (2 * Math.PI), 1.49);
		passingCalculator.loadLUTEntry(11.0, 280.0 / (2 * Math.PI), 1.75); // Adjusted from 17.16/360
		passingCalculator.loadLUTEntry(13.0, 320.0 / (2 * Math.PI), 1.76);
		passingCalculator.loadLUTEntry(17.16, 360.0 / (2 * Math.PI), 2.16);
	}

	public LaunchingParameters getParameters(Pose2d robotPose, ChassisSpeeds robotVelocity) {
		return getParameters(robotPose, robotVelocity, DrivetrainConstants.RED_HUB_POSE.getTranslation(), false);
	}

	public LaunchingParameters getParameters(Pose2d robotPose, ChassisSpeeds robotVelocity, Translation2d target, boolean isPassing) {
		ShotCalculator calc = isPassing ? passingCalculator : hubCalculator;
		
		// ChassisSpeeds is robot-relative, ShotCalculator expects field-relative for fieldVelocity
		// but LaunchCalculator's robotVelocity input is actually ChassisSpeeds (robot-relative)
		// We need to convert it to field-relative for the solver.
		double vxField = robotVelocity.vxMetersPerSecond * robotPose.getRotation().getCos() - robotVelocity.vyMetersPerSecond * robotPose.getRotation().getSin();
		double vyField = robotVelocity.vxMetersPerSecond * robotPose.getRotation().getSin() + robotVelocity.vyMetersPerSecond * robotPose.getRotation().getCos();
		ChassisSpeeds fieldVelocity = new ChassisSpeeds(vxField, vyField, robotVelocity.omegaRadiansPerSecond);

		ShotInputs inputs = new ShotInputs(
				robotPose,
				fieldVelocity,
				robotVelocity,
				target,
				new Translation2d(1, 0), // Default hub forward vector
				1.0 // Vision confidence
		);

		LaunchParameters params = calc.calculate(inputs);

		return new LaunchingParameters(
				params.isValid(),
				params.driveAngle(),
				params.driveAngularVelocityRadPerSec(),
				params.rpm(),
				params.solvedDistanceM(),
				params.timeOfFlightSec(),
				isPassing,
				params.confidence()
		);
	}

	public void clearLaunchingParameters() {
		hubCalculator.resetWarmStart();
		passingCalculator.resetWarmStart();
	}
}
