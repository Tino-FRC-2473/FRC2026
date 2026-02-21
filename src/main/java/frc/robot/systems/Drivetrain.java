package frc.robot.systems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.generated.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;
import frc.robot.input.Input;
import frc.robot.input.InputTypes.ButtonInput;
import frc.robot.input.InputTypes.AxialInput;

public class Drivetrain extends FSMSystem<Drivetrain.DrivetrainState> {
	/* ======================== Constants ======================== */

	// FSM states enum
	public enum DrivetrainState {
		TELEOP
	}

	// Max linear & angular speeds
	private static final LinearVelocity MAX_SPEED = TunerConstants.SPEED_12V;
	private static final AngularVelocity MAX_ANGULAR_SPEED =
		DrivetrainConstants.MAX_ANGULAR_VELOCITY;

	private final AprilTagFieldLayout TAG_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

	// Drive swerve requests
	private final SwerveRequest.FieldCentric driveFieldCentric = new SwerveRequest.FieldCentric()
			.withDeadband(MAX_SPEED.in(MetersPerSecond)
					* DrivetrainConstants.TRANSLATIONAL_DEADBAND)
			.withRotationalDeadband(MAX_ANGULAR_SPEED.in(RadiansPerSecond)
					* DrivetrainConstants.ROTATIONAL_DEADBAND)
			// Use open-loop for drive motors
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

	/* ======================== Private variables ======================== */

	// Current FSM state
	private DrivetrainState currentState;
	// Drivetrain subsystem instance
	private CommandSwerveDrivetrain drivetrain;

	/**
	 * Constructs the drivetrain subsystem.
	 */
	public Drivetrain() {
		drivetrain = TunerConstants.createDrivetrain();

		reset();
	}

	/* ======================== Public methods ======================== */

	@Override
	public void reset() {
		currentState = DrivetrainState.TELEOP;

		update(null);
	}

	@Override
	public void update(Input input) {
		drivetrain.periodic();
		Logger.recordOutput("Target Pose", TAG_LAYOUT.getTagPose(10).get().toPose2d());

		switch (currentState) {
			case TELEOP:
				handleTeleopState(input);
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
	private ShooterFSMSystem shooter;

	private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle =
		new SwerveRequest.FieldCentricFacingAngle()
		.withDeadband(MAX_SPEED.in(MetersPerSecond) * DrivetrainConstants.TRANSLATIONAL_DEADBAND)
		.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

	/**
     * Assigns the shooter FSM instance to the drivetrain to enable state-aware.
     * features like auto-alignment.
     * @param shooter The ShooterFSMSystem instance to be referenced.
     */
	public void setShooter(ShooterFSMSystem shooter) {
		this.shooter = shooter;
	}

	/**
     * Handles manual robot movement during the teleop period, including logic
	 * for field-centric driving and automatic heading alignment based on.
     * current shooter states.
     * @param input The teleop input containing axis and button data.
     */
	private void handleTeleopState(Input input) {
		if (input == null) {
			return;
		}

		double xSpeed = MathUtil.applyDeadband(
			input.getAxisValue(AxialInput.DRIVETRAIN_DRIVE_X),
			DrivetrainConstants.TRANSLATIONAL_DEADBAND) * MAX_SPEED.in(MetersPerSecond);

		double ySpeed = MathUtil.applyDeadband(
			-input.getAxisValue(AxialInput.DRIVETRAIN_DRIVE_Y),
			DrivetrainConstants.TRANSLATIONAL_DEADBAND) * MAX_SPEED.in(MetersPerSecond);

		Pose2d target = null; //(shooter != null) ? shooter.getAutoTargetPose() : null;

		target = TAG_LAYOUT.getTagPose(10).get().toPose2d();


		if (target != null && DriverStation.getAlliance().isPresent()) {
			double angle = Math.atan2(
				target.getY() - getPose().getY(),
				target.getX() - getPose().getX()
			);

			drivetrain.setControl(
				driveFacingAngle
					.withVelocityX(xSpeed * DrivetrainConstants.TRANSLATIONAL_DAMP)
					.withVelocityY(ySpeed * DrivetrainConstants.TRANSLATIONAL_DAMP)
					.withTargetDirection(edu.wpi.first.math.geometry.Rotation2d.fromRadians(angle))
					.withHeadingPID(7, 0, 0)
			);
		} else {
			double thetaSpeed = MathUtil.applyDeadband(
					-input.getAxisValue(AxialInput.DRIVETRAIN_ROTATE),
				DrivetrainConstants.ROTATIONAL_DEADBAND) * MAX_ANGULAR_SPEED.in(RadiansPerSecond);

			drivetrain.setControl(
				driveFieldCentric
					.withVelocityX(-
					xSpeed * DrivetrainConstants.TRANSLATIONAL_DAMP)
					.withVelocityY(ySpeed * DrivetrainConstants.TRANSLATIONAL_DAMP)
					.withRotationalRate(thetaSpeed * DrivetrainConstants.ROTATIONAL_DAMP)
			);
			System.out.println("Hi");
		}

		if (input.getButtonPressed(ButtonInput.DRIVETRAIN_RESEED)) {
			drivetrain.seedFieldCentric();
		}
	}

	@Override
	protected DrivetrainState nextState(Input input) {
		if (input == null) {
			return DrivetrainState.TELEOP;
		}

		switch (currentState) {
			case TELEOP:
				return DrivetrainState.TELEOP;
			default:
				throw new IllegalStateException(
					"[DRIVETRAIN] Cannot get next state of an invalid current state: "
				+ currentState.toString()
				);
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
	 * Aligns the bot to target the hub for shooting.
	 *
	 */
	public void targetHub() {
		Pose2d transformPose = getPose().relativeTo(ShooterConstants.HUB_POSE);
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
            visionPoseMeters.getRotation()),
            timestampSeconds, visionStdDevs);
        //drivetrain.addVisionMeasurement(visionPoseMeters, timestampSeconds,visionStdDevs);
    }
}
