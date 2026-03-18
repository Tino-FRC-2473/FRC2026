package frc.robot.systems;

// WPILib Imports

// Third party Hardware Imports

import frc.robot.HardwareMap;
import frc.robot.motors.TalonFXWrapper;
import frc.robot.input.Input;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import frc.robot.Constants.AgitatorConstants;

public class AgitatorFSMSystem extends FSMSystem<AgitatorFSMSystem.AgitatorFSMState> {

	public enum AgitatorFSMState {
		IDLE,
		SHOOT_CONVEYOR,
		OUTTAKE_CONVEYOR
	}

	/* ======================== Constants ======================== */

	/* ======================== Private variables ======================== */

	private TalonFXWrapper conveyorMotor;
	private MotionMagicVelocityVoltage conveyorMotionRequest;

	private final IntakeFSMSystem intake;
	private final ShooterFSMSystem shooter;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 *
	 * @param intakeSystem  is the intake fsm system
	 * @param shooterSystem is the shooter fsm system
	 */
	public AgitatorFSMSystem(IntakeFSMSystem intakeSystem, ShooterFSMSystem shooterSystem) {
		intake = intakeSystem;
		shooter = shooterSystem;

		var talonFXConfigs = new TalonFXConfiguration();

		var pivotConfig = talonFXConfigs.Feedback;
		pivotConfig.SensorToMechanismRatio = AgitatorConstants.CONVEYOR_GEARING;

		var outputConfigs = talonFXConfigs.MotorOutput;
		outputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

		var slot0Configs = talonFXConfigs.Slot0;
		slot0Configs.kV = AgitatorConstants.CONVEYOR_KV;
		slot0Configs.kA = AgitatorConstants.CONVEYOR_KA;
		slot0Configs.kP = AgitatorConstants.CONVEYOR_KP;
		slot0Configs.kI = AgitatorConstants.CONVEYOR_KI;
		slot0Configs.kD = AgitatorConstants.CONVEYOR_KD;
		slot0Configs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

		// Perform hardware init using a wrapper class
		// this is so we can see motor outputs during simulatiuons
		conveyorMotor = new TalonFXWrapper(HardwareMap.CAN_ID_CONVEYOR);
		conveyorMotionRequest = new MotionMagicVelocityVoltage(0);

		// Reset state machine
		reset();
	}

	/* ======================== Public methods ======================== */

	// overridden methods don't require javadocs
	// however, you may want to add implementation specific javadocs

	@Override
	public void reset() {
		setCurrentState(AgitatorFSMState.IDLE);

		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}

	@Override
	public void update(Input input) {
		if (input == null) {
			return;
		}
		switch (getCurrentState()) {
			case IDLE:
				handleIdleState(input);
				break;

			case SHOOT_CONVEYOR:
				handleShootConveyorState(input);
				break;

			case OUTTAKE_CONVEYOR:
				handleOuttakeConveyorState(input);
				break;

			default:
				throw new IllegalStateException("Invalid state: " + getCurrentState().toString());
		}
		setCurrentState(nextState(input));
	}

	/* ======================== Protected methods ======================== */

	@Override
	protected AgitatorFSMState nextState(Input input) {
		switch (getCurrentState()) {
			case IDLE:
				if (intake.getIsIntakeOuttaking()) {
					return AgitatorFSMState.OUTTAKE_CONVEYOR;
				} else if (shooter.getIsFeeding()) {
					return AgitatorFSMState.SHOOT_CONVEYOR;
				} else {
					return AgitatorFSMState.IDLE;
				}

			case SHOOT_CONVEYOR:
				if (shooter.getIsFeeding()) {
					return AgitatorFSMState.SHOOT_CONVEYOR;
				} else if (intake.getIsIntakeOuttaking() && !shooter.getIsFeeding()) {
					return AgitatorFSMState.OUTTAKE_CONVEYOR;
				} else {
					return AgitatorFSMState.IDLE;
				}

			case OUTTAKE_CONVEYOR:
				if (intake.getIsIntakeOuttaking()) {
					return AgitatorFSMState.OUTTAKE_CONVEYOR;
				} else if (shooter.getIsFeeding() && !intake.getIsIntakeOuttaking()) {
					return AgitatorFSMState.SHOOT_CONVEYOR;
				} else {
					return AgitatorFSMState.IDLE;
				}

			default:
				throw new IllegalStateException("Invalid state: " + getCurrentState().toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in IDLE.
	 *
	 * @param input Global Input if robot in teleop mode or null if
	 *              the robot is in autonomous mode.
	 */
	private void handleIdleState(Input input) {
		conveyorMotor.setControl(conveyorMotionRequest.withVelocity(0));
	}

	/**
	 * Handle behavior in SHOOT_CONVEYOR.
	 *
	 * @param input Global Input if robot in teleop mode or null if
	 *              the robot is in autonomous mode.
	 */
	private void handleShootConveyorState(Input input) {
		conveyorMotor.setControl(conveyorMotionRequest.
			withVelocity(AgitatorConstants.SHOOT_CONVEYOR_TARGET_VELOCITY));
	}

	/**
	 * Handle behavior in OUTTAKE_CONVEYOR.
	 *
	 * @param input Global Input if robot in teleop mode or null if
	 *              the robot is in autonomous mode.
	 */
	private void handleOuttakeConveyorState(Input input) {
		conveyorMotor.setControl(conveyorMotionRequest.
			withVelocity(AgitatorConstants.OUTTAKE_CONVEYOR_TARGET_VELOCITY));
	}

}
