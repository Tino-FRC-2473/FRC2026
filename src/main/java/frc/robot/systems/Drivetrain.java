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
		DrivetrainConstants.MAX_ANGULAR_VELO_RPS;

	// Drive swerve requests
	private final SwerveRequest.FieldCentric driveFieldCentric = new SwerveRequest.FieldCentric()
			.withDeadband(MAX_SPEED.in(MetersPerSecond)
					* DrivetrainConstants.TRANSLATION_DEADBAND)
			.withRotationalDeadband(MAX_ANGULAR_SPEED.in(RadiansPerSecond)
					* DrivetrainConstants.ROTATION_DEADBAND)
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

	/* ======================== Private methods ======================== */

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

	private void handleTeleopState(Input input) {
		if (input == null) {
			return;
		}

		double xSpeed = MathUtil.applyDeadband(
				-input.getAxis(AxialInput.DRIVE_X),
				DrivetrainConstants.TRANSLATION_DEADBAND) * MAX_SPEED.in(MetersPerSecond);

		double ySpeed = MathUtil.applyDeadband(
				-input.getAxis(AxialInput.DRIVE_Y),
				DrivetrainConstants.TRANSLATION_DEADBAND) * MAX_SPEED.in(MetersPerSecond);

		double thetaSpeed = MathUtil.applyDeadband(
				-input.getAxis(AxialInput.ROTATE),
				DrivetrainConstants.ROTATION_DEADBAND) * MAX_ANGULAR_SPEED.in(RadiansPerSecond);

		drivetrain.setControl(
			driveFieldCentric
				.withVelocityX(xSpeed * DrivetrainConstants.TRANSLATIONAL_DAMP)
				.withVelocityY(ySpeed * DrivetrainConstants.TRANSLATIONAL_DAMP)
				.withRotationalRate(thetaSpeed * DrivetrainConstants.ROTATIONAL_DAMP)
		);

		if (input.getButtonPressed(ButtonInput.RESEED_DRIVETRAIN)) {
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

}
