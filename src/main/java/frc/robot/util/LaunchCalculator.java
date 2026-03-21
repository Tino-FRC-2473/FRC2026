package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.firecontrol.ProjectileSimulator;
import frc.robot.util.firecontrol.ProjectileSimulator.LUTEntry;
import frc.robot.util.firecontrol.ProjectileSimulator.SimParameters;
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

		// Physics parameters for simulation
		SimParameters params = new SimParameters(
				ShooterConstants.BALL_MASS_KG,
				ShooterConstants.BALL_DIAMETER_M,
				ShooterConstants.BALL_DRAG_COEFF,
				ShooterConstants.BALL_MAGNUS_COEFF,
				ShooterConstants.AIR_DENSITY,
				ShooterConstants.ROBOT_TO_LAUNCHER.getZ(),
				ShooterConstants.SHOOTER_WHEEL_DIAMETER_M,
				ShooterConstants.TARGET_HEIGHT_M,
				ShooterConstants.SHOOTER_SLIP_FACTOR,
				ShooterConstants.HOOD_ANGLE.in(edu.wpi.first.units.Units.Degrees),
				0.001,  // dt
				500,    // rpmMin
				6000,   // rpmMax
				25,     // binarySearchIters
				5.0     // maxSimTime
		);

		ProjectileSimulator sim = new ProjectileSimulator(params);
		var lut = sim.generateLUT();

		// Load Hub LUT from physics-generated values
		for (LUTEntry entry : lut.entries()) {
			if (entry.reachable()) {
				hubCalculator.loadLUTEntry(entry.distanceM(), entry.rpm(), entry.tof());
			}
		}

		// Load Passing LUT (reusing generated LUT for now as it's physics-based)
		for (LUTEntry entry : lut.entries()) {
			if (entry.reachable()) {
				passingCalculator.loadLUTEntry(entry.distanceM(), entry.rpm(), entry.tof());
			}
		}
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
