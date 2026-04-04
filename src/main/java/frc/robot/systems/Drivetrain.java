package frc.robot.systems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.Constants.DrivetrainConstants.BLUE_ALLIANCE_TAG_26;
import static frc.robot.Constants.DrivetrainConstants.BLUE_HUB_POSE;
import static frc.robot.Constants.DrivetrainConstants.MAX_ANGULAR_SPEED;
import static frc.robot.Constants.DrivetrainConstants.MAX_SPEED;
import static frc.robot.Constants.DrivetrainConstants.RED_ALLIANCE_TAG_10;
import static frc.robot.Constants.DrivetrainConstants.ROTATIONAL_DAMP;
import static frc.robot.Constants.DrivetrainConstants.ROTATIONAL_DEADBAND;
import static frc.robot.Constants.DrivetrainConstants.TRANSLATIONAL_DAMP;
import static frc.robot.Constants.DrivetrainConstants.TRANSLATIONAL_DEADBAND;
import static frc.robot.Constants.DrivetrainConstants.X_TAG_OFFSET;
import static frc.robot.Constants.DrivetrainConstants.Y_TAG_OFFSET;
import static frc.robot.imported.FieldConstants.TAG_LAYOUT;


import java.io.IOException;
import java.sql.Driver;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ModuleConstants;
import frc.robot.generated.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;
import frc.robot.imported.geom.AllianceFlipUtil;
import frc.robot.input.Input;
import frc.robot.input.InputTypes.AxialInput;
import frc.robot.input.InputTypes.ButtonInput;
import frc.robot.Robot;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.imported.LaunchCalculator;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N10;


public class Drivetrain extends FSMSystem<Drivetrain.DrivetrainState> {

	//KEEP THIS BOOLEAN FALSE UNLESS OTHERWISE UNDERSTOOD
	public static boolean USE_SOTM_AIMING = true;

	/* ======================== Enums ======================== */
	// region

	public enum DrivetrainState {
		AUTONOMOUS,
		CONTROLLED,
		PATHFINDING, 
		FACE_HUB,
		FACE_PASS,
		BALL_SHAKE_FRONT,
		BALL_SHAKE_SIDE
	}

	// endregion
	/* ======================== Constants ======================== */
	// region

	private final SwerveRequest.FieldCentric driveFieldCentric = new SwerveRequest.FieldCentric()
			.withDeadband(MAX_SPEED.in(MetersPerSecond) * TRANSLATIONAL_DEADBAND)
			.withRotationalDeadband(MAX_ANGULAR_SPEED.in(RadiansPerSecond) * ROTATIONAL_DEADBAND)
			// Use open-loop for drive motors
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

	private final SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds
		= new SwerveRequest.ApplyRobotSpeeds()
			.withDriveRequestType(DriveRequestType.Velocity);

	private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle
		= new SwerveRequest.FieldCentricFacingAngle()
			.withDeadband(MAX_SPEED.in(MetersPerSecond)
				* DrivetrainConstants.TRANSLATIONAL_DEADBAND)
			.withRotationalDeadband(MAX_ANGULAR_SPEED.in(RadiansPerSecond)
				* DrivetrainConstants.ROTATIONAL_DEADBAND)
			.withDriveRequestType(DriveRequestType.Velocity);

	private final SlewRateLimiter xInputRateLimiter
		= new SlewRateLimiter(DrivetrainConstants.INPUT_SLEW_RATE);
	private final SlewRateLimiter yInputRateLimiter
		= new SlewRateLimiter(DrivetrainConstants.INPUT_SLEW_RATE);
	// private final SlewRateLimiter thetaInputRateLimiter
	// 	= new SlewRateLimiter(DrivetrainConstants.INPUT_SLEW_RATE);

	// endregion
	/* ======================== Private variables ======================== */
	// region

	private DrivetrainState currentState;
	private CommandSwerveDrivetrain drivetrain;
	private Command pathfindCommand = null;
	private Field2d simulatedField = new Field2d();

	private Alliance alliance;
	private boolean invertControls = false;
	private double xSpeed;
	private double ySpeed;
	private double thetaSpeed;
	private Pose2d currentHubPose;
	private int alignmentTargetTag;

	private double targetAngle = 0;
	/**
	 * Construct the drivetrain subsystem.
	 */
	public Drivetrain() {

		createDrivetrain();
		updateSmartDashboard();
		configureAutoBuilder();
		reset();
	}

	// endregion
	/* ======================== Public methods ======================== */
	// region

	protected DrivetrainState getDefaultState() {
		return DrivetrainState.AUTONOMOUS;
	}

	@Override
	public void reset() {
		resetCurrentState();
		grabAlliance();
		stopDrivetrain();
		update(null);
	}

	@Override
	public void update(Input input) {
		drivetrain.periodic();
		CommandScheduler.getInstance().run();
		simulatedField.setRobotPose(drivetrain.getState().Pose);

		switch (currentState) {
			case AUTONOMOUS:
			case PATHFINDING:
				break;
			case CONTROLLED:
				handleTeleopState(input);
				break;
			case FACE_HUB:
				handleFaceTarget(input, getHubPose());
				break;
			case FACE_PASS:
				handleFaceTarget(input, getBestPassingTarget());
				break;
			case BALL_SHAKE_FRONT:
				handleBallShake(input);
				break;
			case BALL_SHAKE_SIDE:
				handleBallShake(input);
				break;
			default:
				throw new IllegalStateException(
						"[DRIVETRAIN] Cannot update an invalid state: " + currentState.toString());
		}

		currentState = nextState(input);
	}

	@Override
	protected DrivetrainState nextState(Input input) {
		if (input == null) {
			return getDefaultState();
		}

		switch (currentState) {
			case AUTONOMOUS:
				if (hasDriverInput(input)) {
					return DrivetrainState.CONTROLLED;
				}
				if (input.getButtonPressed(ButtonInput.FACE_HUB)) {
					return DrivetrainState.FACE_HUB;
				} else if (input.getButtonPressed(ButtonInput.FACE_PASS)) {
					return DrivetrainState.FACE_PASS;
				} else if (input.getButtonPressed(ButtonInput.BALL_SHAKE_FRONT)) {
					return DrivetrainState.BALL_SHAKE_FRONT;
				} else if (input.getButtonPressed(ButtonInput.BALL_SHAKE_SIDE)) {
					return DrivetrainState.BALL_SHAKE_SIDE;
				} else {
					return DrivetrainState.AUTONOMOUS;
				}
			case CONTROLLED:
				if (input.getButtonValue(ButtonInput.BALL_SHAKE_FRONT)) {
					return DrivetrainState.BALL_SHAKE_FRONT;
				} else if (input.getButtonValue(ButtonInput.BALL_SHAKE_SIDE)) {
					return DrivetrainState.BALL_SHAKE_SIDE;
				}

				if (input.getButtonPressed(ButtonInput.DRIVETRAIN_PATHFIND)) {
					if (alliance == null) {
						if (isRedAlliance()) {
							alignmentTargetTag = RED_ALLIANCE_TAG_10;
							System.out.println("RED ALLIANCE TAG 10");
						} else {
							alignmentTargetTag = BLUE_ALLIANCE_TAG_26;
							System.out.println("BLUE ALLIANCE TAG 26");
						}
					} else {
						alignmentTargetTag = RED_ALLIANCE_TAG_10;
						System.out.println("Defaulting to red. Good luck");
					}

					startPathfinding();

					return DrivetrainState.PATHFINDING;
				} else if (input.getButtonPressed(ButtonInput.FACE_HUB)) {
					return DrivetrainState.FACE_HUB;
				} else if (input.getButtonPressed(ButtonInput.FACE_PASS)) {
					return DrivetrainState.FACE_PASS;
				} else {
					return DrivetrainState.CONTROLLED;
				}
			case PATHFINDING:
				if (input.getButtonValue(ButtonInput.DRIVETRAIN_PATHFIND)) {
					return DrivetrainState.PATHFINDING;
				} else {
					pathfindCommand.cancel();
					return DrivetrainState.CONTROLLED;
				}
			case FACE_HUB:
				if (input.getButtonValue(ButtonInput.FACE_HUB)) {
					return DrivetrainState.FACE_HUB;
				} else if (DriverStation.isAutonomous()) {
					return DrivetrainState.AUTONOMOUS;
				} else {
					return DrivetrainState.CONTROLLED;
				}
			case FACE_PASS:
				if (input.getButtonValue(ButtonInput.FACE_PASS)) {
					return DrivetrainState.FACE_PASS;
				} else if (DriverStation.isAutonomous()) {
					return DrivetrainState.AUTONOMOUS;
				} else {
					return DrivetrainState.CONTROLLED;
				}
			case BALL_SHAKE_FRONT:
				if (input.getButtonValue(ButtonInput.BALL_SHAKE_FRONT)) {
					return DrivetrainState.BALL_SHAKE_FRONT;
				} else if (DriverStation.isAutonomous()) {
					return DrivetrainState.AUTONOMOUS;
				} else {
					return DrivetrainState.CONTROLLED;
				}
			case BALL_SHAKE_SIDE:
				if (input.getButtonValue(ButtonInput.BALL_SHAKE_SIDE)) {
					return DrivetrainState.BALL_SHAKE_SIDE;
				} else if (DriverStation.isAutonomous()) {
					return DrivetrainState.AUTONOMOUS;
				} else {
					return DrivetrainState.CONTROLLED;
				}
			default:
				throw new IllegalStateException(
						"[DRIVETRAIN] Cannot get next state from an invalud state: "
								+ currentState.toString());
		}
	}

	/**
	 * Adds a new timestamped vision measurement.
	 *
	 * @param visionPose    the pose of the robot in the camera's coordinate frame
	 * @param timestamp     the timestamp of the measurement
	 * @param visionStdDevs the standard deviations of the measurement in the
	 *                      x, y, and theta directions
	 */
	public void addVisionMeasurement(
			Pose2d visionPose,
			double timestamp,
			Matrix<N3, N1> visionStdDevs) {
		drivetrain.addVisionMeasurement(new Pose2d(
				visionPose.getX(),
				visionPose.getY(),
				visionPose.getRotation().plus(Rotation2d.k180deg)),
				timestamp, visionStdDevs);
	}

	// endregion
	/* ======================== Getset methods ======================== */
	// region

	/**
	 * Get the drivetrain pose.
	 *
	 * @return the pose
	 */
	@AutoLogOutput(key = "Drivetrain/Pose")
	public Pose2d getPose() {
		return drivetrain.getState().Pose;
	}

	/**
	 * Get the drivetrain rotation.
	 *
	 * @return the rotation
	 */
	@AutoLogOutput(key = "Drivetrain/Rotation")
	public Rotation3d getRotation() {
		return drivetrain.getPigeon2().getRotation3d();
	}

	/**
	 * Get the drivetrain rotation.
	 *
	 * @return the rotation
	 */
	@AutoLogOutput(key = "Drivetrain/RotationDouble")
	public double getRotationDouble() {
		return MathUtil.angleModulus(drivetrain.getPigeon2()
			.getRotation2d().getRadians() + Math.PI);
	}

	/**
	 * Get the drivetrain swerve states.
	 *
	 * @return the swerve module states
	 */
	@AutoLogOutput(key = "Drivetrain/Swerve/States")
	public SwerveModuleState[] getSwerveStates() {
		return drivetrain.getState().ModuleStates;
	}

	/**
	 * Get the drivetrain swerve targets.
	 *
	 * @return the swerve module targets
	 */
	@AutoLogOutput(key = "Drivetrain/Swerve/Targets")
	public SwerveModuleState[] getSwerveTargets() {
		return drivetrain.getState().ModuleTargets;
	}

	/**
	 * Get the drivetrain swerve speeds.
	 *
	 * @return the swerve module speeds
	 */
	@AutoLogOutput(key = "Drivetrain/Swerve/Speeds")
	public ChassisSpeeds getSwerveSpeeds() {
		return drivetrain.getState().Speeds;
	}

	/**
	 * Get the drivetrain swerve positions.
	 *
	 * @return the swerve module positions
	 */
	@AutoLogOutput(key = "Drivetrain/Swerve/Positions")
	public SwerveModulePosition[] getSwervePositions() {
		return drivetrain.getState().ModulePositions;
	}

	/**
	 * Get the current state of the Drivetrain subsystem.
	 *
	 * @return the current state of the drivetrain
	 */
	@AutoLogOutput(key = "Drivetrain/Current State")
	public DrivetrainState getCurrentState() {
		return currentState;
	}

	/**
	 * Get the current pathfinding target.
	 *
	 * @return the current pathfinding target
	 */
	//@AutoLogOutput(key = "Vision/Alignment Pose")
	public Pose2d getPathfindingTarget() {
		return TAG_LAYOUT.getTagPose(alignmentTargetTag)
				.orElse(null)
				.toPose2d()
				.transformBy(new Transform2d(
					X_TAG_OFFSET,
					Y_TAG_OFFSET,
					Rotation2d.kZero
				));
	}

	/**
	 * Get the current hub pose.
	 *
	 * @return the current hub pose
	 */
	@AutoLogOutput(key = "Drivetrain/Current Hub Pose")
	public Pose2d getCurrentHubPose() {
		return AllianceFlipUtil.apply(DrivetrainConstants.BLUE_HUB_POSE);
	}

	/**
	 * Get the current chassis speed.
	 *
	 * @return the current chassis speeds
	 */
	@AutoLogOutput(key = "Drivetrain/Swerve/Chassis Speeds")
	public ChassisSpeeds getChassisSpeeds() {
		return drivetrain.getState().Speeds;
	}

	// endregion
	/* ======================== Private methods ======================== */
	// region

	private void createDrivetrain() {
		drivetrain = TunerConstants.createDrivetrain();
		driveFacingAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
	}

	public Pose2d getHubPose() {
		var currentAlliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
		return (currentAlliance == DriverStation.Alliance.Red) ? DrivetrainConstants.RED_HUB_POSE
				: DrivetrainConstants.BLUE_HUB_POSE;
	}

	private Pose2d getBestPassingTarget() {
		var currentAlliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
		boolean isRed = (currentAlliance == DriverStation.Alliance.Red);

		Pose2d outpost = isRed ? DrivetrainConstants.RED_OUTPOST_POSE
			: DrivetrainConstants.BLUE_OUTPOST_POSE;
		Pose2d pose3 = isRed ? DrivetrainConstants.RED_POSE3_POSE
			: DrivetrainConstants.BLUE_POSE3_POSE;

		double distOutpost = getPose().getTranslation().getDistance(outpost.getTranslation());
		double distPose3 = getPose().getTranslation().getDistance(pose3.getTranslation());

		return (distOutpost < distPose3) ? outpost : pose3;
	}

	private void updateSmartDashboard() {
		SmartDashboard.putData(CommandScheduler.getInstance());
		SmartDashboard.putData(simulatedField);
	}

	private void configureAutoBuilder() {
		RobotConfig config = null;
		try {
			config = RobotConfig.fromGUISettings();
		} catch (IOException | ParseException e) {
		}

		AutoBuilder.configure(
				this::getPose, // Robot pose supplier
				// Method to reset odometry (will be called if your auto has a starting pose)
				drivetrain::resetPose,
				() -> {
					return drivetrain.getState().Speeds;
				}, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
				(speeds, feedforwards) -> {

					ChassisSpeeds speedINeedThis = new ChassisSpeeds(
							speeds.vxMetersPerSecond,
							speeds.vyMetersPerSecond,
							-speeds.omegaRadiansPerSecond);

					drivetrain.setControl(
							applyRobotSpeeds
									.withSpeeds(speedINeedThis.times(TRANSLATIONAL_DAMP))
									.withWheelForceFeedforwardsX(
											feedforwards.robotRelativeForcesXNewtons())
									.withWheelForceFeedforwardsY(
											feedforwards.robotRelativeForcesYNewtons()));

				}, /*
					 * Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also
					 * optionally outputs individual module feedforwards
					 */
				new PPHolonomicDriveController(/*
												 * PPHolonomicController is the built in path
												 * following controller for holonomic drive trains
												 */
						// Translation PID constants
						new PIDConstants(ModuleConstants.DRIVE_P,
								ModuleConstants.DRIVE_I, ModuleConstants.DRIVE_D),
						// Rotation PID constants
						new PIDConstants(ModuleConstants.STEER_P,
								ModuleConstants.STEER_I, ModuleConstants.STEER_D)),
				config, // The robot configuration
				() -> {
					/*
					 * Boolean supplier that controls when the
					 * path will be mirrored for the red alliance
					 */
					// This will flip the path being followed to the red side of the field.
					// THE ORIGIN WILL REMAIN ON THE BLUE SIDE

					return !Robot.IS_BLUE;

					// var detectedAlliance = DriverStation.getAlliance();
					// if (detectedAlliance.isPresent()) {
					// 	return detectedAlliance.get() == DriverStation.Alliance.Red;
					// }
					// System.out.println("Alliance not detected! Defaulting to blue. Good luck.");
					// return false;
				},
				drivetrain // Reference to the subsystem to set requirements
		);
	}

	private void resetCurrentState() {
		currentState = getDefaultState();
	}

	private void grabAlliance() {
		alliance = DriverStation.getAlliance().orElse(null);
	}

	private void stopDrivetrain() {
		drivetrain.applyRequest(() -> driveFieldCentric
				.withVelocityX(0)
				.withVelocityY(0)
				.withRotationalRate(0));
	}

	private boolean hasDriverInput(Input input) {
		return input.getAxisValue(AxialInput.DRIVETRAIN_DRIVE_X) != 0
				|| input.getAxisValue(AxialInput.DRIVETRAIN_DRIVE_Y) != 0
				|| input.getAxisValue(AxialInput.DRIVETRAIN_ROTATE) != 0;
	}

	/**
	 * Get the drivetrain swerve states.
	 *
	 * @return the swerve module states
	 */
	@AutoLogOutput(key = "Drivetrain/Swerve/States")
	public SwerveModuleState[] getModuleStates() {
		return drivetrain.getState().ModuleStates;
	}

	/**
	 * Get the drivetrain swerve targets.
	 *
	 * @return the swerve module targets
	 */
	@AutoLogOutput(key = "Drivetrain/Swerve/Targets")
	public SwerveModuleState[] getModuleTargets() {
		return drivetrain.getState().ModuleTargets;
	}

	/**
	 * Get the drivetrain swerve positions.
	 *
	 * @return the swerve module positions
	 */
	@AutoLogOutput(key = "Drivetrain/Swerve/Positions")
	public SwerveModulePosition[] getModulePositions() {
		return drivetrain.getState().ModulePositions;
	}

	/**
	 * Get the drivetrain's rotation.
	 *
	 * @return The drivetrain's rotation as a Pose2D
	 */
	@AutoLogOutput(key = "Drivetrain/Rotation")
	public Rotation3d getDrivetrainRotation() {
		return drivetrain.getPigeon2().getRotation3d();
	}

	/**
	 * Get the drivetrain's angular velocity.
	 *
	 * @return The drivetrain's angular velocity in dps.
	 */
	public double getAngularVelocity() {
		return drivetrain.getPigeon2().getAngularVelocityXDevice().getValueAsDouble();
	}

	/**
	 * Gets the linear velocity of the robot by averaging motor encoders.
	 * Use this to check against VisionConstants.MAX_LINEAR_SPEED.
	 * @return calculated linear velocity
	 */
	public double getLinearVelocityFromEncoders() {
		// 1. Calculate conversion (Wheel Circ / Gear Ratio)
		double conversion = (Math.PI * DrivetrainConstants.METERS_TO_INCHES)
			/ DrivetrainConstants.DRIVETRAIN_MOTOR_GEARING;

		// 2. Get speeds from all 4 drive motors (Rotations per Second)
		double fl = drivetrain.getModule(0).getEncoder().getVelocity().getValueAsDouble();
		double fr = drivetrain.getModule(1).getEncoder().getVelocity().getValueAsDouble();
		double bl = drivetrain.getModule(2).getEncoder().getVelocity().getValueAsDouble();
		double br = drivetrain.getModule(2 + 1).getEncoder().getVelocity().getValueAsDouble();

		// 3. Average the motor speeds and convert to m/s
		double avgMotorSpeed = (fl + fr + bl + br) / (2 + 2);

		return Math.abs(avgMotorSpeed * conversion);
	}
	private void startPathfinding() {
		// Pose2d target = getPathfindingTarget();
		// pathfindCommand = AutoBuilder.pathfindToPose(target, PATH_CONSTRAINTS);
		CommandScheduler.getInstance().schedule(pathfindCommand);
	}

	private void handleFaceTarget(Input input, Pose2d targetPose) {
		if (input == null) {
			return;
		}

		handleDriveInputs(input);

		double flipAlliance = 0

		if (!Robot.IS_BLUE) {
			flipAlliance = Math.PI;
		}

		if (USE_SOTM_AIMING) {
			boolean isPassing = (getCurrentState() == DrivetrainState.FACE_PASS);
			var params = LaunchCalculator.getInstance().getParameters(
				getPose(),
				getChassisSpeeds(),
				targetPose.getTranslation(),
				isPassing
			);
			targetAngle = MathUtil.angleModulus(params.driveAngle().getRadians() + flipAlliance);
			//pigeon odomtery is backwards

			Logger.recordOutput("Drivetrain/Error", Math.abs(MathUtil.angleModulus(
				getRotationDouble() - targetAngle + flipAlliance)));
			
			Logger.recordOutput("Drivetrain/Target", targetAngle);
			Logger.recordOutput("Drivetrain/Rotation", getRotationDouble());

			// Pure targeting: stay locked perfectly on target without giving up at 10 degrees,
			// otherwise the robot will drift out of aim!

			if (Math.abs(MathUtil.angleModulus(getRotationDouble() - targetAngle)) > Math.toRadians(DrivetrainConstants.FACE_TARGET_DEADBAND)) {
				drivetrain.setControl(
					driveFacingAngle
							.withTargetDirection(Rotation2d.fromRadians(targetAngle))
							.withHeadingPID(DrivetrainConstants.FACE_HUB_P,
								DrivetrainConstants.FACE_HUB_I, DrivetrainConstants.FACE_HUB_D)
							.withVelocityX(xSpeed * DrivetrainConstants.TRANSLATIONAL_DAMP)
							.withVelocityY(ySpeed * DrivetrainConstants.TRANSLATIONAL_DAMP));
			} else {
				drivetrain.setControl(
						driveFieldCentric
								.withVelocityX(xSpeed * DrivetrainConstants.TRANSLATIONAL_DAMP)
								.withVelocityY(ySpeed * DrivetrainConstants.TRANSLATIONAL_DAMP)
								//.withRotationalRate(thetaSpeed * DrivetrainConstants.ROTATIONAL_DAMP));
								.withRotationalRate(0));
			}

		} else {

			// Transform2d distance = targetPose.minus(getPose());
			// Using angleModulus to prevent wrap-around issues with filtering
			Translation2d robotTranslation = getPose().getTranslation();
			Translation2d targetTranslation = targetPose.getTranslation();
			Translation2d diff = targetTranslation.minus(robotTranslation);
			targetAngle = Math.atan2(diff.getY(), diff.getX());
			// double currentAngle = getRotation();
			// double rawTarget = Math.atan2(distance.getY(), distance.getX());

			// double error = MathUtil.angleModulus(rawTarget - currentAngle);

			// if(Math.abs(error) > Math.PI / 2) {
			// rawTarget = MathUtil.angleModulus(rawTarget + Math.PI);
			// }

			// targetAngle = MathUtil.angleModulus(rawTarget);

			// double currentAngle = drivetrain.getState().Pose.getRotation().getRadians();
			// double angleError = Math.abs(MathUtil.angleModulus(targetAngle -
			// currentAngle));
			Logger.recordOutput("Drivetrain/Error", Math.abs(MathUtil.angleModulus(
				getRotationDouble() - targetAngle - Math.PI / 2)));

			if (Math.abs(MathUtil.angleModulus(
				getRotationDouble() - targetAngle - Math.PI / 2))
				> Math.toRadians(DrivetrainConstants.FACE_TARGET_DEADBAND)) {
				// System.out.println("aaa");
				drivetrain.setControl(
						driveFacingAngle
								.withTargetDirection(Rotation2d.fromRadians(targetAngle))
								.withHeadingPID(DrivetrainConstants.FACE_HUB_P,
									DrivetrainConstants.FACE_HUB_I, DrivetrainConstants.FACE_HUB_D)
								.withVelocityX(xSpeed * DrivetrainConstants.TRANSLATIONAL_DAMP)
								.withVelocityY(ySpeed * DrivetrainConstants.TRANSLATIONAL_DAMP)
				);
			} else {

				drivetrain.setControl(
						driveFieldCentric
								.withVelocityX(xSpeed * DrivetrainConstants.TRANSLATIONAL_DAMP)
								.withVelocityY(ySpeed * DrivetrainConstants.TRANSLATIONAL_DAMP)
								//.withRotationalRate(thetaSpeed * DrivetrainConstants.ROTATIONAL_DAMP));
								.withRotationalRate(0));
			}
		}
	}

	private void handleBallShake(Input input) {
		//calculate how much to change position by
		double shakeSpeed = Math.sin(edu.wpi.first.wpilibj.Timer.getFPGATimestamp()
						* DrivetrainConstants.SHAKE_FREQUENCY * 2.0 * Math.PI)
						* DrivetrainConstants.SHAKE_MAGNITUDE;

		//Apply the shakey shakey
		if (getCurrentState() == DrivetrainState.BALL_SHAKE_FRONT) {
			drivetrain.setControl(
				applyRobotSpeeds.withSpeeds(new ChassisSpeeds(shakeSpeed, 0, 0))
			);
		} else {
			drivetrain.setControl(
				applyRobotSpeeds.withSpeeds(new ChassisSpeeds(0, shakeSpeed, 0))
			);
		}
	}

	private void handleTeleopState(Input input) {
		if (input == null) {
			return;
		}

		currentHubPose = getCurrentHubPose();

		handleDriveInputs(input);
		applyDriveControl();

		handleButtonInputs(input);
	}

	private void handleDriveInputs(Input input) {
		xSpeed = getTranslationalSpeed(input, AxialInput.DRIVETRAIN_DRIVE_X);
		ySpeed = getTranslationalSpeed(input, AxialInput.DRIVETRAIN_DRIVE_Y);
		thetaSpeed = getRotationalSpeed(input, AxialInput.DRIVETRAIN_ROTATE);
	}

	private double getTranslationalSpeed(Input input, AxialInput inputType) {
		SlewRateLimiter limiter = switch (inputType) {
			case DRIVETRAIN_DRIVE_X -> xInputRateLimiter;
			case DRIVETRAIN_DRIVE_Y -> yInputRateLimiter;
			default -> null;
		};

		if (limiter == null) {
			return 0;
		}

		double speed = MathUtil.applyDeadband(
				input.getAxisValue(inputType),
				TRANSLATIONAL_DEADBAND) * MAX_SPEED.in(MetersPerSecond);

		speed = limiter.calculate(speed);

		if (invertControls) {
			speed *= -1;
		}

		if (!Robot.IS_BLUE) {
			speed *= -1;
		}

		return speed;
	}

	private boolean isRedAlliance() {
		return alliance == DriverStation.Alliance.Red;
	}

	private double getRotationalSpeed(Input input, AxialInput inputType) {
		double rotationalSpeed = MathUtil.applyDeadband(
				input.getAxisValue(inputType),
				ROTATIONAL_DEADBAND) * MAX_ANGULAR_SPEED.in(RadiansPerSecond);
		return rotationalSpeed;
		//return thetaInputRateLimiter.calculate(rotationalSpeed);
	}

	private void applyDriveControl() {
		drivetrain.setControl(
				driveFieldCentric
						.withVelocityX(xSpeed * TRANSLATIONAL_DAMP)
						.withVelocityY(ySpeed * TRANSLATIONAL_DAMP)
						.withRotationalRate(thetaSpeed * ROTATIONAL_DAMP));
	}

	private void handleButtonInputs(Input input) {
		handleInvertControlsButton(input);
		handleReseedButton(input);
	}

	private void handleInvertControlsButton(Input input) {
		if (input.getButtonPressed(ButtonInput.DRIVETRAIN_INVERT_CONTROLS)) {
			invertControls = !invertControls;
		}
	}

	private void handleReseedButton(Input input) {
		if (input.getButtonPressed(ButtonInput.DRIVETRAIN_RESEED)) {
			drivetrain.seedFieldCentric();
		}
	}

	/**
	 * Stops the drivetrain.
	 */
	public void stop() {
		drivetrain.applyRequest(
				() -> driveFieldCentric.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
	}

	/**
	 * Follows a path with the given name.
	 *
	 * @param name
	 * @return Command
	 */
	public Command followcommand(String name) {
		System.out.println(AutoBuilder.isConfigured());
		PathPlannerPath path;
		try {
			path = PathPlannerPath.fromChoreoTrajectory(name);
		} catch (IOException | ParseException e) {
			DriverStation.reportError("PathPlanner Error: Failed to load path: " + name
					+ ". Check if the path exists and is properly configured.", e.getStackTrace());
			return Commands.none();
		}
		if (name == null) {
			DriverStation.reportError("PathPlanner Error: Path is null",
				new Exception().getStackTrace());
			return Commands.none();
		}
		return AutoBuilder.followPath(path);
	}



	// endregion
	/* ======================== End ======================== */
}
