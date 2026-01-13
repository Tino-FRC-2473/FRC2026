package frc.robot.systems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

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
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.TeleopInput;
import frc.robot.generated.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;
import frc.robot.systems.AutoHandlerSystem.AutoFSMState;

public class Drivetrain extends FSMSystem<Drivetrain.DrivetrainState> {
	public enum DrivetrainState {
		TELEOP
	}

	private static final LinearVelocity MAX_SPEED = TunerConstants.SPEED_AT_12_VOLTS;

	// kSpeedAt12Volts desired top speed
	private static final AngularVelocity MAX_ANGULAR_RATE =
		DrivetrainConstants.MAX_ANGULAR_VELO_RPS;
	// 3/4 rps angle velo

	/* ======================== Private variables ======================== */
	private DrivetrainState currentState;
	private CommandSwerveDrivetrain drivetrain;

	private final SwerveRequest.FieldCentric driveFieldCentric = new SwerveRequest.FieldCentric()
			.withDeadband(MAX_SPEED.in(MetersPerSecond)
					* DrivetrainConstants.TRANSLATION_DEADBAND) // 4% deadband
			.withRotationalDeadband(MAX_ANGULAR_RATE.in(RadiansPerSecond)
					* DrivetrainConstants.ROTATION_DEADBAND) // 4% deadband
			// Use open-loop for drive motors
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

	private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

	/**
	 * Constructs the drivetrain subsystem.
	 */
	public Drivetrain() {
		drivetrain = TunerConstants.createDrivetrain();

		reset();
	}

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

	@Override
	protected DrivetrainState nextState(TeleopInput input) {
		if (input == null) {
			return DrivetrainState.TELEOP;
		}

		switch (currentState) {
			case TELEOP:
				return DrivetrainState.TELEOP;
			default:
				throw new IllegalStateException(
					"[DRIVETRAIN] Invalid Current State for Next State: " + currentState.toString()
				);
		}
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
				DrivetrainConstants.ROTATION_DEADBAND) * MAX_ANGULAR_RATE.in(RadiansPerSecond);

		drivetrain.setControl(
			driveFieldCentric
				.withVelocityX(xSpeed * DrivetrainConstants.TRANSLATIONAL_DAMP)
				.withVelocityY(ySpeed * DrivetrainConstants.TRANSLATIONAL_DAMP)
				.withRotationalRate(thetaSpeed * DrivetrainConstants.ROTATIONAL_DAMP)
		);
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
	 * Get the pose of the drivetrain.
	 *
	 * @return pose of the drivetrain
	 */
	@AutoLogOutput(key = "Drivetrain/Pose")
	public Pose2d getPose() {
		return drivetrain.getState().Pose;
	}

	/**
	 * Get the chassis speeds of the drivetrain.
	 *
	 * @return the drivetrain chassis speeds
	 */
	@AutoLogOutput(key = "Drivetrain/Swerve/Chassis Speeds")
	public ChassisSpeeds getChassisSpeeds() {
		return drivetrain.getState().Speeds;
	}

	/**
	 * Get the drivetrain states.
	 *
	 * @return the swerve module states
	 */
	@AutoLogOutput(key = "Drivetrain/Swerve/States/Measured")
	public SwerveModuleState[] getModuleStates() {
		return drivetrain.getState().ModuleStates;
	}

	/**
	 * Get the drivetrain targets.
	 *
	 * @return drivetrain targets
	 */
	@AutoLogOutput(key = "Drivetrain/Swerve/States/Targets")
	public SwerveModuleState[] getModuleTargets() {
		return drivetrain.getState().ModuleTargets;
	}

	/**
	 * Get the drivetrain module positions.
	 *
	 * @return the module positions
	 */
	@AutoLogOutput(key = "Drivetrain/Swerve/Positions")
	public SwerveModulePosition[] getModulePositions() {
		return drivetrain.getState().ModulePositions;
	}
}
