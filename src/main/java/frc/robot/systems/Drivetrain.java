package frc.robot.systems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.TeleopInput;
import frc.robot.generated.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;
import frc.robot.systems.AutoHandlerSystem.AutoFSMState;

public class Drivetrain extends FSMSystem<Drivetrain.DrivetrainState> {
	/* ======================== Constants ======================== */

	// FSM states enum
	public enum DrivetrainState {
		TELEOP,
		PATHFIND
	}

	// Max linear & angular speeds
	private static final LinearVelocity MAX_SPEED = TunerConstants.SPEED_12V;
	private static final AngularVelocity MAX_ANGULAR_SPEED =
		DrivetrainConstants.MAX_ANGULAR_VELO_RPS;

	// Drive swerve requests
	private final SwerveRequest.FieldCentric driveFieldCentric = new SwerveRequest.FieldCentric()
			.withDeadband(MAX_SPEED.in(MetersPerSecond)
					* DrivetrainConstants.TRANSLATION_DEADBAND)
			.withRotationalDeadband(MAX_ANGULAR_SPEED.in(RadiansPerSecond)
					* DrivetrainConstants.ROTATION_DEADBAND)
			// Use open-loop for drive motors
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

	private final SwerveRequest.ApplyRobotSpeeds
			applyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds()
		.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

	/* ======================== Private variables ======================== */

	// Current FSM state
	private DrivetrainState currentState;
	// Drivetrain subsystem instance
	private CommandSwerveDrivetrain drivetrain;
	//Pathfind command
	private Command pathfindCommand = null;
	//Pathfind target
	private Pose2d pathfindTarget = new Pose2d();

	/**
	 * Constructs the drivetrain subsystem.
	 */
	public Drivetrain() {
		drivetrain = TunerConstants.createDrivetrain();

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
					drivetrain.setControl(
						applyRobotSpeeds
							.withSpeeds(speeds)
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

				var alliance = DriverStation.getAlliance();
				if (alliance.isPresent()) {
					return alliance.get() == DriverStation.Alliance.Red;
				}
				return false;
				},
				drivetrain // Reference to the subsystem to set requirements
		);

		reset();
	}

	/* ======================== Public methods ======================== */

	@Override
	public void reset() {
		currentState = DrivetrainState.TELEOP;

		update(null);
	}

	@Override
	public void update(TeleopInput input) {
		drivetrain.periodic();

		switch (currentState) {
			case TELEOP:
				handleTeleopState(input);
				break;
			case PATHFIND:
				//No need to do anything? You only start/stop pathfinding
				break;
			default:
				throw new IllegalStateException(
					"[DRIVETRAIN] Invalid Current State: " + currentState.toString()
				);
		}

		currentState = nextState(input);
	}

	@Override
	public boolean updateAutonomous(AutoFSMState autoState) {
		return false;
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
	protected DrivetrainState nextState(TeleopInput input) {
		if (input == null) {
			return DrivetrainState.TELEOP;
		}

		switch (currentState) {
			case TELEOP:
				if (input.isPathfindButtonPressed()) {
					startPathfinding();
					return DrivetrainState.PATHFIND;
				} else {
					return DrivetrainState.TELEOP;
				}
			case PATHFIND:
				if (input.isPathfindButtonPressed()) {
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

	private void startPathfinding() {
		pathfindCommand = AutoBuilder.pathfindToPose(pathfindTarget,
					DrivetrainConstants.PATH_CONSTRAINTS);
		CommandScheduler.getInstance().schedule(pathfindCommand);
	}

	private void handleTeleopState(TeleopInput input) {
		if (input == null) {
			return;
		}

		double xSpeed = MathUtil.applyDeadband(
				-input.getDriverLeftY(),
				DrivetrainConstants.TRANSLATION_DEADBAND) * MAX_SPEED.in(MetersPerSecond);

		double ySpeed = MathUtil.applyDeadband(
				-input.getDriverLeftX(),
				DrivetrainConstants.TRANSLATION_DEADBAND) * MAX_SPEED.in(MetersPerSecond);

		double thetaSpeed = MathUtil.applyDeadband(
				-input.getDriverRightX(),
				DrivetrainConstants.ROTATION_DEADBAND) * MAX_ANGULAR_SPEED.in(RadiansPerSecond);

		drivetrain.setControl(
			driveFieldCentric
				.withVelocityX(xSpeed * DrivetrainConstants.TRANSLATIONAL_DAMP)
				.withVelocityY(ySpeed * DrivetrainConstants.TRANSLATIONAL_DAMP)
				.withRotationalRate(thetaSpeed * DrivetrainConstants.ROTATIONAL_DAMP)
		);

		if (input.isDriverReseedButtonPressed()) {
			drivetrain.seedFieldCentric();
		}
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
		drivetrain.addVisionMeasurement(
				visionPoseMeters,
				timestampSeconds,
				visionStdDevs);
	}

}
