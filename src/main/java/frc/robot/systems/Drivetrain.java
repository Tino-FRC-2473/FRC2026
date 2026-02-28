package frc.robot.systems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
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
		TELEOP,
		FACE_HUB,
		FACE_PASS_ZONE
	}

	// Max linear & angular speeds
	private static final LinearVelocity MAX_SPEED = TunerConstants.SPEED_12V;
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

	/* ======================== Private variables ======================== */

	// Current FSM state
	private DrivetrainState currentState;
	// Drivetrain subsystem instance
	private CommandSwerveDrivetrain drivetrain;
	private ShooterFSMSystem shooter;

	/**
	 * Constructs the drivetrain subsystem.
	 * @param shooterFSMSystem the shooter FSM system, used for targeting the hub in FACE_HUB state
	 */
	public Drivetrain(Optional<ShooterFSMSystem> shooterFSMSystem) {
		drivetrain = TunerConstants.createDrivetrain();
		shooter = shooterFSMSystem.orElse(null);
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

		switch (currentState) {
			case TELEOP:
				handleTeleopState(input);
				break;
			case FACE_HUB:
				handleFaceHubState(input);
				break;
			case FACE_PASS_ZONE:
				handleFacePassZoneState(input);
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

	/* ======================== Private methods ======================== */

	private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle =
		new SwerveRequest.FieldCentricFacingAngle()
		.withDeadband(MAX_SPEED.in(MetersPerSecond) * DrivetrainConstants.TRANSLATIONAL_DEADBAND)
		.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

	@Override
	protected DrivetrainState nextState(Input input) {
		if (input == null) {
			return DrivetrainState.TELEOP;
		}

		switch (currentState) {
			case TELEOP:
				if (input.getButtonPressed(ButtonInput.FACE_SHOOTER)) {
					if (shooter.getIsPastStateShooterPrep()
						|| shooter.getIsCurrentStateShooterPrep()) {
						return DrivetrainState.FACE_HUB;
					} else {
						return DrivetrainState.FACE_PASS_ZONE;
					}
				}
				return DrivetrainState.TELEOP;
			case FACE_HUB:
				if (input.getButtonReleased(ButtonInput.FACE_SHOOTER)) {
					return DrivetrainState.TELEOP;
				}
				return DrivetrainState.FACE_HUB;
			case FACE_PASS_ZONE:
				if (input.getButtonReleased(ButtonInput.FACE_SHOOTER)) {
					return DrivetrainState.TELEOP;
				}
				return DrivetrainState.FACE_PASS_ZONE;
			default:
				throw new IllegalStateException(
					"[DRIVETRAIN] Cannot get next state of an invalid current state: "
					+ currentState.toString()
				);
		}
	}

	private void handleFaceHubState(Input input) {
		if (input == null) {
			return;
		}

		double xSpeed = MathUtil.applyDeadband(
			input.getAxisValue(AxialInput.DRIVETRAIN_DRIVE_X),
			DrivetrainConstants.TRANSLATIONAL_DEADBAND) * MAX_SPEED.in(MetersPerSecond);

		double ySpeed = MathUtil.applyDeadband(
			-input.getAxisValue(AxialInput.DRIVETRAIN_DRIVE_Y),
			DrivetrainConstants.TRANSLATIONAL_DEADBAND) * MAX_SPEED.in(MetersPerSecond);

		Pose2d targetPose;
		if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get()
			== DriverStation.Alliance.Red) {
			targetPose = DrivetrainConstants.RED_HUB_POSE;
		} else {
			targetPose = DrivetrainConstants.BLUE_HUB_POSE;
		}


		if (targetPose != null && DriverStation.getAlliance().isPresent()) {
			double angle = Math.atan2(
				targetPose.getY() - getPose().getY(),
				targetPose.getX() - getPose().getX()
			);

			drivetrain.setControl(
				driveFacingAngle
					.withVelocityX(xSpeed * DrivetrainConstants.TRANSLATIONAL_DAMP)
					.withVelocityY(ySpeed * DrivetrainConstants.TRANSLATIONAL_DAMP)
					.withTargetDirection(edu.wpi.first.math.geometry.Rotation2d.fromRadians(angle))
					.withHeadingPID(DrivetrainConstants.FACE_HUB_P,
						DrivetrainConstants.FACE_HUB_I,
						DrivetrainConstants.FACE_HUB_D)
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
		}

		if (input.getButtonPressed(ButtonInput.DRIVETRAIN_RESEED)) {
			drivetrain.seedFieldCentric();
		}
	}

	private void handleFacePassZoneState(Input input) {
		if (input == null) {
			return;
		}

		double xSpeed = MathUtil.applyDeadband(
			input.getAxisValue(AxialInput.DRIVETRAIN_DRIVE_X),
			DrivetrainConstants.TRANSLATIONAL_DEADBAND) * MAX_SPEED.in(MetersPerSecond);

		double ySpeed = MathUtil.applyDeadband(
			-input.getAxisValue(AxialInput.DRIVETRAIN_DRIVE_Y),
			DrivetrainConstants.TRANSLATIONAL_DEADBAND) * MAX_SPEED.in(MetersPerSecond);

		Pose2d targetPose;
		if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get()
			== DriverStation.Alliance.Red) {
			targetPose = DrivetrainConstants.RED_PASSING_POSE;
		} else {
			targetPose = DrivetrainConstants.BLUE_PASSING_POSE;
		}


		if (targetPose != null && DriverStation.getAlliance().isPresent()) {
			double angle = Math.atan2(
				targetPose.getY() - getPose().getY(),
				targetPose.getX() - getPose().getX()
			);

			drivetrain.setControl(
				driveFacingAngle
					.withVelocityX(xSpeed * DrivetrainConstants.TRANSLATIONAL_DAMP)
					.withVelocityY(ySpeed * DrivetrainConstants.TRANSLATIONAL_DAMP)
					.withTargetDirection(edu.wpi.first.math.geometry.Rotation2d.fromRadians(angle))
					.withHeadingPID(DrivetrainConstants.FACE_HUB_P,
						DrivetrainConstants.FACE_HUB_I,
						DrivetrainConstants.FACE_HUB_D)
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
		}

		if (input.getButtonPressed(ButtonInput.DRIVETRAIN_RESEED)) {
			drivetrain.seedFieldCentric();
		}
	}



	private void handleTeleopState(Input input) {
		if (input == null) {
			return;
		}

		double xSpeed = MathUtil.applyDeadband(
				-input.getAxisValue(AxialInput.DRIVETRAIN_DRIVE_X),
				DrivetrainConstants.TRANSLATIONAL_DEADBAND) * MAX_SPEED.in(MetersPerSecond);

		double ySpeed = MathUtil.applyDeadband(
				-input.getAxisValue(AxialInput.DRIVETRAIN_DRIVE_Y),
				DrivetrainConstants.TRANSLATIONAL_DEADBAND) * MAX_SPEED.in(MetersPerSecond);

		double thetaSpeed = MathUtil.applyDeadband(
				-input.getAxisValue(AxialInput.DRIVETRAIN_ROTATE),
				DrivetrainConstants.ROTATIONAL_DEADBAND) * MAX_ANGULAR_SPEED.in(RadiansPerSecond);

		drivetrain.setControl(
			driveFieldCentric
				.withVelocityX(xSpeed * DrivetrainConstants.TRANSLATIONAL_DAMP)
				.withVelocityY(ySpeed * DrivetrainConstants.TRANSLATIONAL_DAMP)
				.withRotationalRate(thetaSpeed * DrivetrainConstants.ROTATIONAL_DAMP)
		);

		if (input.getButtonPressed(ButtonInput.DRIVETRAIN_RESEED)) {
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
	 * Aligns the bot to target the hub for shooting.
	 *
	 */
	public void targetHub() {
		//Code to be finished in a seperate branch
		Pose2d transformPose = getPose().relativeTo(ShooterConstants.HUB_POSE);
	}

	/**
	 * Aligns the bot to target the passing target.
	 * @param targetPose the target passing pose
	 */
	public void targetPassZone(Pose2d targetPose) {
		//toggleNumber = 1 for outpost, toggleNumber = 2 for other mirrored pose, etc
		//Code to be finished in a seperate branch
		Pose2d transformPose = getPose().relativeTo(targetPose);
		//Code to be implemented differently later
	}
}
