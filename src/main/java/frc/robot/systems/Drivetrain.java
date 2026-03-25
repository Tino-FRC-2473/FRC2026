package frc.robot.systems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.Constants.DrivetrainConstants.BLUE_HUB_POSE;
import static frc.robot.Constants.DrivetrainConstants.MAX_ANGULAR_SPEED;
import static frc.robot.Constants.DrivetrainConstants.MAX_SPEED;
import static frc.robot.Constants.DrivetrainConstants.PATH_CONSTRAINTS;
import static frc.robot.Constants.DrivetrainConstants.ROTATIONAL_DEADBAND;
import static frc.robot.Constants.DrivetrainConstants.TRANSLATIONAL_DEADBAND;
import static frc.robot.imported.FieldConstants.TAG_LAYOUT;

import java.io.IOException;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

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
import edu.wpi.first.math.numbers.N10;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N6;
import edu.wpi.first.math.numbers.N7;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.generated.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;
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

	private boolean invertControls = false;

	private Field2d simulatedField = new Field2d();
	private Pose2d currentHubPose;

	/**
	 * Construct the drivetrain subsystem.
	 */
	public Drivetrain() throws IOException, ParseException {
		createDrivetrain();
		updateSmartDashboard();
		configureAutoBuilder();
		reset();
	}

	// endregion
	/* ======================== Public methods ======================== */
	// region

	@Override
	public void reset() {
		resetCurrentState();
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
			return DrivetrainState.CONTROLLED;
		}

		switch (currentState) {
			case AUTONOMOUS:
				if (hasDriverInput(input)) {
					return DrivetrainState.CONTROLLED;
				}
				return DrivetrainState.AUTONOMOUS;
			case CONTROLLED:
				if (input.getButtonPressed(ButtonInput.DRIVETRAIN_PATHFIND)) {

					if (DriverStation.getAlliance().isPresent()) {

						if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
							DrivetrainConstants.setTagToAlignTo(N10.instance.getNum());
							System.out.println("RED ALLIANCE TAG 10");
						} else {
							DrivetrainConstants.setTagToAlignTo(N10.instance.getNum()
								+ N10.instance.getNum()
								+ N6.instance.getNum());
							System.out.println("BLUE ALLIANCE TAG 26");
						}

					} else {
						System.out.println("Defaulting to red. Good luck");
						DrivetrainConstants.setTagToAlignTo(N10.instance.getNum());
					}

					Pose2d test = TAG_LAYOUT.getTagPose(
						DrivetrainConstants.getTagToAlignTo()).orElse(null).toPose2d();

					Transform2d offsetTransform = new Transform2d(
							-DrivetrainConstants.X_TAG_OFFSET,
							DrivetrainConstants.Y_TAG_OFFSET,
							Rotation2d.kZero);

					startPathfinding(test.transformBy(offsetTransform));
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
					"[DRIVETRAIN] Cannot get next state of an invalid current state: "
					+ currentState.toString()
				);
		}
	}

	/**
	 * Adds a new timestamped vision measurement.
	 *
	 * @param visionPoseMeters The pose of the robot in the camera's coordinate
	 *                         frame
	 * @param timestampSeconds The timestamp of the measurement
	 * @param visionStdDevs    The standard deviations of the measurement in the x,
	 *                         y, and theta directions
	 */
	public void addVisionMeasurement(
			Pose2d visionPoseMeters,
			double timestampSeconds,
			Matrix<N3, N1> visionStdDevs) {
		drivetrain.addVisionMeasurement(new Pose2d(visionPoseMeters.getX(),
			visionPoseMeters.getY(),
			visionPoseMeters.getRotation().plus(Rotation2d.k180deg)),
			timestampSeconds, visionStdDevs);
		//drivetrain.addVisionMeasurement(visionPoseMeters, timestampSeconds,visionStdDevs);
	}

	/**
	 * Returns the target rotation needed given a target pose.
	 * @param target the target pose
	 * @return the rotation
	 */
	public Rotation2d getTargetHub(Pose2d target) {
		Transform2d distance = target.minus(getPose());
		return Rotation2d.fromRadians(Math.atan2(distance.getY(), distance.getX()))
				.rotateBy(Rotation2d.kCCW_90deg);
	}

	/**
	 * Aligns the bot to target the passing target.
	 * @param targetPose the target passing pose
	 */
	public void targetPassZone(Pose2d targetPose) {
		//toggleNumber = 1 for outpost, toggleNumber = 2 for other mirrored pose, etc
		//TODO: Code to be finished in a seperate branch
		Pose2d transformPose = getPose().relativeTo(targetPose);
		//TODO: Code to be implemented differently later
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
	 * @return current state of the drivetrain
	 */
	@AutoLogOutput(key = "Drivetrain/Current State")
	public DrivetrainState getCurrentState() {
		return currentState;
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

	private void configureAutoBuilder() throws IOException, ParseException {
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
						.withSpeeds(speedINeedThis.times(
							Constants.DrivetrainConstants.TRANSLATIONAL_DAMP))
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
			RobotConfig.fromGUISettings(), // The robot configuration
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
		currentState = DrivetrainState.AUTONOMOUS;
	}

	private void stopDrivetrain() {
		drivetrain.applyRequest(() -> driveFieldCentric
				.withVelocityX(0)
				.withVelocityY(0)
				.withRotationalRate(0)
		);
	}

	private void startPathfinding(Pose2d target) {

		Logger.recordOutput("Vision/AlignmentPose", target);

		pathfindCommand = AutoBuilder.pathfindToPose(target,
					PATH_CONSTRAINTS);
		CommandScheduler.getInstance().schedule(pathfindCommand);
	}

	private void handleTeleopState(Input input) {
		if (input == null) {
			return;
		}

		if (input.getButtonPressed(ButtonInput.DRIVETRAIN_INVERT_CONTROLS)) {
			invertControls = !invertControls;
		}

		currentHubPose = AllianceFlipUtil.apply(BLUE_HUB_POSE);
		Logger.recordOutput("Hub Pose Alignment", currentHubPose);

		//TODO: Clean this jawn up it's for testing
		double flipAlliance = -1;
		var alliance = DriverStation.getAlliance();
		if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
			flipAlliance = 1.0;
		}
		double xSpeed = flipAlliance * MathUtil.applyDeadband(
				input.getAxisValue(AxialInput.DRIVETRAIN_DRIVE_X),
				DrivetrainConstants.TRANSLATIONAL_DEADBAND) * MAX_SPEED.in(MetersPerSecond);

		double ySpeed = flipAlliance * MathUtil.applyDeadband(
				input.getAxisValue(AxialInput.DRIVETRAIN_DRIVE_Y),
				DrivetrainConstants.TRANSLATIONAL_DEADBAND) * MAX_SPEED.in(MetersPerSecond);

		double thetaSpeed = MathUtil.applyDeadband(
				input.getAxisValue(AxialInput.DRIVETRAIN_ROTATE),
				DrivetrainConstants.ROTATIONAL_DEADBAND) * MAX_ANGULAR_SPEED.in(RadiansPerSecond);

		if (invertControls) {
			xSpeed *= -1;
			ySpeed *= -1;
		}

		drivetrain.setControl(
			driveFieldCentric
					.withVelocityX(xSpeed * DrivetrainConstants.TRANSLATIONAL_DAMP)
					.withVelocityY(ySpeed * DrivetrainConstants.TRANSLATIONAL_DAMP)
					.withRotationalRate(thetaSpeed * DrivetrainConstants.ROTATIONAL_DAMP)
		);

		if (input.getButtonPressed(ButtonInput.DRIVETRAIN_RESEED)) {
			drivetrain.seedFieldCentric();
		}

		if (input.getButtonValue(ButtonInput.FACE_HUB)) {
			Transform2d distance = getPose().minus(currentHubPose);
			double angle = Math.atan2(distance.getY(), distance.getX());
			drivetrain.setControl(
				driveFacingAngle
					.withTargetDirection(edu.wpi.first.math.geometry.Rotation2d.fromRadians(angle))
					.withHeadingPID(N7.instance.getNum(), 0, 0)
					.withVelocityX(xSpeed * DrivetrainConstants.TRANSLATIONAL_DAMP)
					.withVelocityY(ySpeed * DrivetrainConstants.TRANSLATIONAL_DAMP)
			);
		}

		if (input.getButtonValue(ButtonInput.FACE_PASS)) {
			double outpostDistance;
			double target3Distance;
			boolean isRed = true;
			Pose2d correctTarget;
			if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
				outpostDistance = Math.sqrt(
						Math.pow(getPose().minus(DrivetrainConstants.RED_OUTPOST_POSE).getX(), 2)
						+ Math.pow(getPose().minus(DrivetrainConstants.RED_OUTPOST_POSE).getY(), 2)
					);
				target3Distance = Math.sqrt(
					Math.pow(getPose().minus(DrivetrainConstants.RED_POSE3_POSE).getX(), 2)
					+ Math.pow(getPose().minus(DrivetrainConstants.RED_POSE3_POSE).getY(), 2)
				);
			} else {
				outpostDistance = Math.sqrt(
					Math.pow(getPose().minus(DrivetrainConstants.BLUE_OUTPOST_POSE).getX(), 2)
						+ Math.pow(getPose().minus(DrivetrainConstants.BLUE_OUTPOST_POSE).getY(), 2)
					);
				target3Distance = Math.sqrt(
					Math.pow(getPose().minus(DrivetrainConstants.BLUE_POSE3_POSE).getX(), 2)
					+ Math.pow(getPose().minus(DrivetrainConstants.BLUE_POSE3_POSE).getY(), 2)
				);
				isRed = false;
			}

			if (outpostDistance < target3Distance) {
				if (isRed) {
					correctTarget = DrivetrainConstants.RED_OUTPOST_POSE;
				} else {
					correctTarget = DrivetrainConstants.BLUE_OUTPOST_POSE;
				}
			} else {
				if (isRed) {
					correctTarget = DrivetrainConstants.RED_POSE3_POSE;
				} else {
					correctTarget = DrivetrainConstants.BLUE_POSE3_POSE;
				}
			}

			Transform2d distance = getPose().minus(correctTarget);
			double angle = Math.atan2(distance.getY(), distance.getX());

			drivetrain.setControl(
				driveFacingAngle
					.withTargetDirection(edu.wpi.first.math.geometry.Rotation2d.fromRadians(angle))
					.withHeadingPID(N7.instance.getNum(), 0, 0)
					.withVelocityX(xSpeed * DrivetrainConstants.TRANSLATIONAL_DAMP)
					.withVelocityY(ySpeed * DrivetrainConstants.TRANSLATIONAL_DAMP)
			);
		}
	}

	private boolean hasDriverInput(Input input) {
		return input.getAxisValue(AxialInput.DRIVETRAIN_DRIVE_X) != 0
			|| input.getAxisValue(AxialInput.DRIVETRAIN_DRIVE_Y) != 0
			|| input.getAxisValue(AxialInput.DRIVETRAIN_ROTATE) != 0;
	}

	// endregion
	/* ======================== End ======================== */
}
