package frc.robot.systems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.Constants.DrivetrainConstants.BLUE_ALLIANCE_TAG_26;
import static frc.robot.Constants.DrivetrainConstants.BLUE_HUB_POSE;
import static frc.robot.Constants.DrivetrainConstants.BLUE_OUTPOST_POSE;
import static frc.robot.Constants.DrivetrainConstants.BLUE_TARGET3_POSE;
import static frc.robot.Constants.DrivetrainConstants.FACE_HUB_D;
import static frc.robot.Constants.DrivetrainConstants.FACE_HUB_I;
import static frc.robot.Constants.DrivetrainConstants.FACE_HUB_P;
import static frc.robot.Constants.DrivetrainConstants.FACE_PASS_D;
import static frc.robot.Constants.DrivetrainConstants.FACE_PASS_I;
import static frc.robot.Constants.DrivetrainConstants.FACE_PASS_P;
import static frc.robot.Constants.DrivetrainConstants.MAX_ANGULAR_SPEED;
import static frc.robot.Constants.DrivetrainConstants.MAX_SPEED;
import static frc.robot.Constants.DrivetrainConstants.PATH_CONSTRAINTS;
import static frc.robot.Constants.DrivetrainConstants.RED_ALLIANCE_TAG_10;
import static frc.robot.Constants.DrivetrainConstants.RED_OUTPOST_POSE;
import static frc.robot.Constants.DrivetrainConstants.RED_TARGET3_POSE;
import static frc.robot.Constants.DrivetrainConstants.ROTATIONAL_DAMP;
import static frc.robot.Constants.DrivetrainConstants.ROTATIONAL_DEADBAND;
import static frc.robot.Constants.DrivetrainConstants.TRANSLATIONAL_DAMP;
import static frc.robot.Constants.DrivetrainConstants.TRANSLATIONAL_DEADBAND;
import static frc.robot.Constants.DrivetrainConstants.X_TAG_OFFSET;
import static frc.robot.Constants.DrivetrainConstants.Y_TAG_OFFSET;
import static frc.robot.imported.FieldConstants.TAG_LAYOUT;

import java.io.IOException;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ModuleConstants;
import frc.robot.generated.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;
import frc.robot.imported.LaunchCalculator;
import frc.robot.imported.geom.AllianceFlipUtil;
import frc.robot.input.Input;
import frc.robot.input.InputTypes.AxialInput;
import frc.robot.input.InputTypes.ButtonInput;


public class Drivetrain extends FSMSystem<Drivetrain.DrivetrainState> {

	/* ======================== Enums ======================== */
	// region

	public enum DrivetrainState {
		AUTONOMOUS,
		CONTROLLED,
		PATHFINDING
	}

	// endregion
	/* ======================== Constants ======================== */
	// region

	private final SwerveRequest.FieldCentric driveFieldCentric = new SwerveRequest.FieldCentric()
			.withDeadband(MAX_SPEED.in(MetersPerSecond) * TRANSLATIONAL_DEADBAND)
			.withRotationalDeadband(MAX_ANGULAR_SPEED.in(RadiansPerSecond) * ROTATIONAL_DEADBAND)
			// Use open-loop for drive motors
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

	private final SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds = new SwerveRequest
			.ApplyRobotSpeeds()
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

	private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new SwerveRequest
			.FieldCentricFacingAngle()
			.withDeadband(MAX_SPEED.in(MetersPerSecond) * TRANSLATIONAL_DEADBAND)
			.withRotationalDeadband(MAX_ANGULAR_SPEED.in(RadiansPerSecond) * ROTATIONAL_DEADBAND)
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

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
			case AUTONOMOUS: case PATHFINDING:
				break;
			case CONTROLLED:
				handleTeleopState(input);
				break;
			default:
				throw new IllegalStateException(
					"[DRIVETRAIN] Cannot update an invalid state: " + currentState.toString()
				);
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
				return DrivetrainState.AUTONOMOUS;
			case CONTROLLED:
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
			default:
				throw new IllegalStateException(
					"[DRIVETRAIN] Cannot get next state from an invalud state: "
						+ currentState.toString()
				);
		}
	}

	/**
	 * Adds a new timestamped vision measurement.
	 *
	 * @param visionPose the pose of the robot in the camera's coordinate frame
	 * @param timestamp the timestamp of the measurement
	 * @param visionStdDevs the standard deviations of the measurement in the
	 * 					   x, y, and theta directions
	 */
	public void addVisionMeasurement(
			Pose2d visionPose,
			double timestamp,
			Matrix<N3, N1> visionStdDevs
	) {
		drivetrain.addVisionMeasurement(new Pose2d(
				visionPose.getX(),
				visionPose.getY(),
				visionPose.getRotation().plus(Rotation2d.k180deg)
			),
			timestamp, visionStdDevs
		);
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
	@AutoLogOutput(key = "Vision/Alignment Pose")
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
		return AllianceFlipUtil.apply(BLUE_HUB_POSE);
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
	}

	private void updateSmartDashboard() {
		SmartDashboard.putData(CommandScheduler.getInstance());
		SmartDashboard.putData(simulatedField);
	}

	private void configureAutoBuilder() {
		RobotConfig config = null;
		try {
			config = RobotConfig.fromGUISettings();
		} catch (IOException | ParseException e) { }

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
						.withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
						.withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
				);

			}, /* Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also
			optionally outputs individual module feedforwards*/
			new PPHolonomicDriveController(/*PPHolonomicController is the built in path
				following controller for holonomic drive trains */
				// Translation PID constants
				new PIDConstants(ModuleConstants.DRIVE_P,
					ModuleConstants.DRIVE_I, ModuleConstants.DRIVE_D),
				// Rotation PID constants
				new PIDConstants(ModuleConstants.STEER_P,
					ModuleConstants.STEER_I, ModuleConstants.STEER_D)
			),
			config, // The robot configuration
			() -> {
				/* Boolean supplier that controls when the
				path will be mirrored for the red alliance*/
				// This will flip the path being followed to the red side of the field.
				// THE ORIGIN WILL REMAIN ON THE BLUE SIDE

				// var alliance = DriverStation.getAlliance();
				// if (alliance.isPresent()) {
				// 	return alliance.get() == DriverStation.Alliance.Red;
				// }
				return false;
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
				.withRotationalRate(0)
		);
	}

	private boolean hasDriverInput(Input input) {
		return input.getAxisValue(AxialInput.DRIVETRAIN_DRIVE_X) != 0
				|| input.getAxisValue(AxialInput.DRIVETRAIN_DRIVE_Y) != 0
				|| input.getAxisValue(AxialInput.DRIVETRAIN_ROTATE) != 0;
	}

	private void startPathfinding() {
		Pose2d target = getPathfindingTarget();
		pathfindCommand = AutoBuilder.pathfindToPose(target, PATH_CONSTRAINTS);
		CommandScheduler.getInstance().schedule(pathfindCommand);
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
		double speed = MathUtil.applyDeadband(
			input.getAxisValue(inputType),
			TRANSLATIONAL_DEADBAND
		) * MAX_SPEED.in(MetersPerSecond);

		if (isRedAlliance()) {
			speed *= -1;
		}

		if (invertControls) {
			speed *= -1;
		}

		return speed;
	}

	private boolean isRedAlliance() {
		return alliance == DriverStation.Alliance.Red;
	}

	private double getRotationalSpeed(Input input, AxialInput inputType) {
		return MathUtil.applyDeadband(
			input.getAxisValue(inputType),
			ROTATIONAL_DEADBAND
		) * MAX_ANGULAR_SPEED.in(RadiansPerSecond);
	}

	private void applyDriveControl() {
		drivetrain.setControl(
			driveFieldCentric
					.withVelocityX(xSpeed * TRANSLATIONAL_DAMP)
					.withVelocityY(ySpeed * TRANSLATIONAL_DAMP)
					.withRotationalRate(thetaSpeed * ROTATIONAL_DAMP)
		);
	}

	private void handleButtonInputs(Input input) {
		handleInvertControlsButton(input);
		handleReseedButton(input);
		handleFaceHubButton(input);
		handleFacePassButton(input);
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

	private void handleFaceHubButton(Input input) {
		if (input.getButtonValue(ButtonInput.FACE_HUB)) {
			drivetrain.setControl(driveFacingAngle
					.withTargetDirection(Rotation2d.fromRadians(getAngleToHub()))
					.withHeadingPID(FACE_HUB_P, FACE_HUB_I, FACE_HUB_D)
					.withVelocityX(xSpeed * TRANSLATIONAL_DAMP)
					.withVelocityY(ySpeed * TRANSLATIONAL_DAMP)
			);
		}
	}

	private double getAngleToHub() {
		Transform2d offset = getPose().minus(currentHubPose);
		return Math.atan2(offset.getY(), offset.getX());
	}

	private void handleFacePassButton(Input input) {
		if (input.getButtonValue(ButtonInput.FACE_PASS)) {
			double outpostDistance = getOutpostDistance();
			double target3Distance = getTarget3Distance();

			Pose2d correctTarget;
			if (outpostDistance < target3Distance) {
				correctTarget = getTargetPose(RED_OUTPOST_POSE, BLUE_OUTPOST_POSE);
			} else {
				correctTarget = getTargetPose(BLUE_TARGET3_POSE, RED_TARGET3_POSE);
			}

			double angle = LaunchCalculator.getInstance()
				.getParameters(getPose(), getChassisSpeeds(), correctTarget.getTranslation(), true)
				.driveAngle().getRadians();

			drivetrain.setControl(
				driveFacingAngle
					.withTargetDirection(Rotation2d.fromRadians(getAngleToPose(correctTarget)))
					.withHeadingPID(FACE_PASS_P, FACE_PASS_I, FACE_PASS_D)
					.withVelocityX(xSpeed * TRANSLATIONAL_DAMP)
					.withVelocityY(ySpeed * TRANSLATIONAL_DAMP)
			);
		}
	}

	private double getOutpostDistance() {
		return getDistanceToPose(getTargetPose(RED_OUTPOST_POSE, BLUE_OUTPOST_POSE));
	}

	private double getTarget3Distance() {
		return getDistanceToPose(getTargetPose(RED_TARGET3_POSE, BLUE_TARGET3_POSE));
	}

	private double getDistanceToPose(Pose2d pose) {
		Transform2d offset = getOffsetToPose(pose);
		return Math.hypot(offset.getX(), offset.getY());
	}

	private double getAngleToPose(Pose2d pose) {
		Transform2d offset = getOffsetToPose(pose);
		return Math.atan2(offset.getY(), offset.getX());
	}

	private Transform2d getOffsetToPose(Pose2d pose) {
		return getPose().minus(pose);
	}

	private Pose2d getTargetPose(Pose2d redPose, Pose2d bluePose) {
		if (isRedAlliance()) {
			return redPose;
		}
		return bluePose;
	}

	// endregion
	/* ======================== End ======================== */
}
