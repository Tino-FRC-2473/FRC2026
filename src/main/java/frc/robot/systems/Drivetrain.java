package frc.robot.systems;


import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.lang.reflect.Field;
import java.util.Optional;
import java.util.Queue;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
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
import edu.wpi.first.units.measure.AngularVelocity;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.generated.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;
import frc.robot.input.Input;
import frc.robot.input.InputTypes.ButtonInput;
import frc.robot.input.InputTypes.AxialInput;


import static frc.robot.Constants.DrivetrainConstants.PATH_CONSTRAINTS;;


public class Drivetrain extends FSMSystem<Drivetrain.DrivetrainState> {
	/* ======================== Constants ======================== */

	// FSM states enum
	public enum DrivetrainState {
		TELEOP,
		PATHFIND,
		ENTRY // so the robot isn't controlled in auto
	}

	// Max linear & angular speeds
	private static final LinearVelocity MAX_SPEED = TunerConstants.kSpeedAt12Volts;
	private static final AngularVelocity MAX_ANGULAR_SPEED =
		DrivetrainConstants.MAX_ANGULAR_VELOCITY;

	// Drive swerve requests
	private final SwerveRequest.FieldCentric driveFieldCentric = new SwerveRequest.FieldCentric()
			.withDeadband(MAX_SPEED.in(MetersPerSecond)
					* DrivetrainConstants.TRANSLATIONAL_DEADBAND)
			.withRotationalDeadband(MAX_ANGULAR_SPEED.in(RadiansPerSecond)
					* DrivetrainConstants.ROTATIONAL_DEADBAND)
			// Use open-loop for drive motors
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

	private final SwerveRequest.ApplyRobotSpeeds
			applyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds()
		.withDriveRequestType(DriveRequestType.OpenLoopVoltage);
	
	private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle =
		new SwerveRequest.FieldCentricFacingAngle()
		.withDeadband(MAX_SPEED.in(MetersPerSecond) * DrivetrainConstants.TRANSLATIONAL_DEADBAND)
		.withRotationalDeadband(MAX_ANGULAR_SPEED.in(RadiansPerSecond)
					* DrivetrainConstants.ROTATIONAL_DEADBAND)
		.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

	/* ======================== Private variables ======================== */

	// Current FSM state
	private DrivetrainState currentState;
	// Drivetrain subsystem instance
	private CommandSwerveDrivetrain drivetrain;
	//Pathfind command
	private Command pathfindCommand = null;
	//Flip controls if needed
	private double invertControls = 1;

	private AprilTagFieldLayout field = AprilTagFieldLayout
			.loadField(AprilTagFields.k2026RebuiltWelded);

	private Field2d elasticfield = new Field2d();

	//TODO: Need to clean this stuff up and put it in constants
	//TODO: Should I call CommandScheduler.getInstance().run(); in a different method
	//instead of the drivetrain's periodic?
	//Pathfind targeting stuff

	//private Pose2d pathfindTarget;
	private ShooterFSMSystem shooter;

	/**
	 * Constructs the drivetrain subsystem.
	 * @param shooterFSMSystem the shooter FSM system, used for targeting the hub in FACE_HUB state
	 */
	public Drivetrain() {
		drivetrain = TunerConstants.createDrivetrain();
		//updateLimelightYaw();

		SmartDashboard.putData(CommandScheduler.getInstance());
		SmartDashboard.putData(elasticfield);

		//System.out.println(DriverStation.getAlliance());


		RobotConfig config;
		try {
			config = RobotConfig.fromGUISettings();
		} catch (Exception e) {
			// Handle exception as needed
			e.printStackTrace();
			throw new RuntimeException(e);
		}

		// Configure AutoBuilder last
		AutoBuilder.configure(
				this::getPose, // Robot pose supplier
				drivetrain::resetPose, /*Method to reset odometry
				(will be called if your auto has a starting pose) */
				() -> {
					return drivetrain.getState().Speeds;
				}, /*ChassisSpeeds supplier. MUST BE ROBOT RELATIVE */
				(speeds, feedforwards) -> {

					ChassisSpeeds speedINeedThis = new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, -speeds.omegaRadiansPerSecond);

					drivetrain.setControl(
						applyRobotSpeeds
							.withSpeeds(speedINeedThis.times(Constants.DrivetrainConstants.TRANSLATIONAL_DAMP))
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

		//shooter = shooterFSMSystem.orElse(null);
		reset();
	}

	/* ======================== Public methods ======================== */

	@Override
	public void reset() {
		currentState = DrivetrainState.ENTRY;
		stop();

		update(null);
	}

	@Override
	public void update(Input input) {
		drivetrain.periodic();
		CommandScheduler.getInstance().run();
		elasticfield.setRobotPose(drivetrain.getState().Pose);

		switch (currentState) {
			case TELEOP:
				handleTeleopState(input);
				break;
			case ENTRY: case PATHFIND:
				break;
			default:
				throw new IllegalStateException(
					"[DRIVETRAIN] Cannot update an invalid current state: "
					+ currentState.toString()
				);
		}

		currentState = nextState(input);
	}

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
	 * Get the drivetrain chassis speeds.
	 *
	 * @return the chassis speeds
	 */
	@AutoLogOutput(key = "Drivetrain/Swerve/Chassis Speeds")
	public ChassisSpeeds getChassisSpeeds() {
		return drivetrain.getState().Speeds;
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

	/* ======================== Private methods ======================== */

	@Override
	protected DrivetrainState nextState(Input input) {
		if (input == null) {
			return DrivetrainState.TELEOP;
		}

		if (input.getButtonPressed(ButtonInput.INVERT_DRIVETRAIN_CONTROLS)) {
			invertControls *= -1;
		}

		switch (currentState) {
			case ENTRY:
				if (hasDriverInput(input)) {
					return DrivetrainState.TELEOP;
				}
				return DrivetrainState.ENTRY;
			case TELEOP:
				if (input.getButtonPressed(ButtonInput.DRIVETRAIN_PATHFIND)) {

					if (DriverStation.getAlliance().isPresent()) {

						if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
							DrivetrainConstants.TAG_TO_ALIGN_TO = 10;
							System.out.println("RED ALLIANCE TAG 10");
						} else {
							DrivetrainConstants.TAG_TO_ALIGN_TO = 26;
							System.out.println("BLUE ALLIANCE TAG 26");
						}

					} else {
						System.out.println("Defaulting to red. Good luck");
						DrivetrainConstants.TAG_TO_ALIGN_TO = 10;
					}

					Pose2d test = field.getTagPose(DrivetrainConstants.TAG_TO_ALIGN_TO).orElse(null).toPose2d();

					Transform2d offsetTransform = new Transform2d(
							-DrivetrainConstants.X_TRANFORM_FROM_TAG, // Back to Front
							DrivetrainConstants.Y_TRANFORM_FROM_TAG, // Side to Side
							Rotation2d.kZero);

					startPathfinding(test.transformBy(offsetTransform));
					return DrivetrainState.PATHFIND;
				} else {
					return DrivetrainState.TELEOP;
				}
			case PATHFIND:
				if (input.getButtonValue(ButtonInput.DRIVETRAIN_PATHFIND)) {
					return DrivetrainState.PATHFIND;
				} else {
					pathfindCommand.cancel();
					return DrivetrainState.TELEOP;
				}
			default:
				throw new IllegalStateException(
					"[DRIVETRAIN] Cannot get next state of an invalid current state: "
					+ currentState.toString()
				);
		}
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

		//TODO: Clean this jawn up it's for testing
		double flipAlliance = -1;
		var alliance = DriverStation.getAlliance();
		if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
			flipAlliance = 1.0;
		}
		double xSpeed = -invertControls * flipAlliance * MathUtil.applyDeadband(
				input.getAxisValue(AxialInput.DRIVETRAIN_DRIVE_Y),
				DrivetrainConstants.TRANSLATION_DEADBAND) * MAX_SPEED.in(MetersPerSecond);

		double ySpeed = -invertControls * flipAlliance * MathUtil.applyDeadband(
				input.getAxisValue(AxialInput.DRIVETRAIN_DRIVE_X),
				DrivetrainConstants.TRANSLATION_DEADBAND) * MAX_SPEED.in(MetersPerSecond);

		double thetaSpeed = MathUtil.applyDeadband(
				input.getAxisValue(AxialInput.DRIVETRAIN_ROTATE),
				DrivetrainConstants.ROTATIONAL_DEADBAND) * MAX_ANGULAR_SPEED.in(RadiansPerSecond);

		drivetrain.setControl(
			driveFieldCentric
				.withVelocityX(ySpeed * DrivetrainConstants.TRANSLATIONAL_DAMP)
				.withVelocityY(xSpeed * DrivetrainConstants.TRANSLATIONAL_DAMP)
				.withRotationalRate(thetaSpeed * DrivetrainConstants.ROTATIONAL_DAMP)
		);

		if (input.getButtonPressed(ButtonInput.DRIVETRAIN_RESEED)) {
			drivetrain.seedFieldCentric();
		}

		if (input.getButtonValue(ButtonInput.FACE_HUB)) {
			Pose2d hubPose;
			if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
				hubPose = DrivetrainConstants.RED_HUB_POSE;
			} else {
				hubPose = DrivetrainConstants.BLUE_HUB_POSE;
			}
			double angle = Math.atan2(
				hubPose.getY() - getPose().getY(),
				hubPose.getX() - getPose().getX()
			);
			if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
				angle *= -1;
			}
			drivetrain.setControl(
				driveFacingAngle
					.withTargetDirection(edu.wpi.first.math.geometry.Rotation2d.fromRadians(angle))
					.withHeadingPID(7, 0, 0)
					.withVelocityX(ySpeed * DrivetrainConstants.TRANSLATIONAL_DAMP)
					.withVelocityY(xSpeed * DrivetrainConstants.TRANSLATIONAL_DAMP)
			);
		}

		if (input.getButtonValue(ButtonInput.FACE_PASS)){
			double outpostDistance;
			double target3Distance;
			boolean isRed = true;
			Pose2d correctTarget;
			if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
				outpostDistance = (double) getPose().getTranslation()
				.getDistance(DrivetrainConstants.RED_OUTPOST_POSE.getTranslation());
				target3Distance = (double) getPose().getTranslation()
				.getDistance(DrivetrainConstants.RED_POSE3_POSE.getTranslation());
			} else {
				outpostDistance = (double) getPose().getTranslation()
				.getDistance(DrivetrainConstants.BLUE_OUTPOST_POSE.getTranslation());
				target3Distance = (double) getPose().getTranslation()
				.getDistance(DrivetrainConstants.BLUE_POSE3_POSE.getTranslation());
				isRed = false;
			}

			if (outpostDistance < target3Distance) {
				if (isRed){
					correctTarget = DrivetrainConstants.RED_OUTPOST_POSE;
				} else {
					correctTarget = DrivetrainConstants.BLUE_OUTPOST_POSE;
				}
			} else {
				if (isRed){
					correctTarget = DrivetrainConstants.RED_POSE3_POSE;
				} else {
					correctTarget = DrivetrainConstants.BLUE_POSE3_POSE;
				}
			}

			double angle = Math.atan2(
				correctTarget.getY() - getPose().getY(),
				correctTarget.getX() - getPose().getX()
			);
			if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
				angle *= -1;
			}
			
			drivetrain.setControl(
				driveFacingAngle
					.withTargetDirection(edu.wpi.first.math.geometry.Rotation2d.fromRadians(angle))
					.withHeadingPID(7, 0, 0)
					.withVelocityX(ySpeed * DrivetrainConstants.TRANSLATIONAL_DAMP)
					.withVelocityY(xSpeed * DrivetrainConstants.TRANSLATIONAL_DAMP)
			);
		}
	}

	/**
	 * Get the current state of the Drivetrain subsystem.
	 *
	 * @return current state of the drivetrain
	 */
	//@AutoLogOutput(key = "Drivetrain/Current State")
	public DrivetrainState getCurrentState() {
		return currentState;
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
	 * Aligns the bot to target the hub for shooting.
	 *
	 */
	public void targetHub() {
		//TODO: Code to be finished in a separate branch
		Pose2d transformPose = getPose().relativeTo(ShooterConstants.HUB_POSE);
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

	private boolean hasDriverInput(Input input) {
		return input.getAxisValue(AxialInput.DRIVETRAIN_DRIVE_X) != 0
			|| input.getAxisValue(AxialInput.DRIVETRAIN_DRIVE_Y) != 0
			|| input.getAxisValue(AxialInput.DRIVETRAIN_ROTATE) != 0;
	}

	public void stop() {
		drivetrain.applyRequest(() -> driveFieldCentric.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
	}
}
