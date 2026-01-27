package frc.robot.systems;


import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.HardwareMap;
import frc.robot.TeleopInput;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.ClimberConstants;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import static edu.wpi.first.units.Units.Inches;



//import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
//import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class ClimberFSMSystem {
	public enum ClimberFSMState {
		IDLE,
		MANUAL_DIRECT_CONTROL,
		L1_EXTEND,
		L1_RETRACT,
		AUTO_UP_1,
		AUTO_UP_2,
		AUTO_DOWN_1,
		AUTO_DOWN_2,
		LOCKED_FINAL
	}

	private final TalonFX climberMotorLeft;
	private final TalonFX climberMotorRight;
	private final DigitalInput groundLimitSwitchLeft;
	private final DigitalInput groundLimitSwitchRight;

	private ClimberFSMState currentState;
	private MotionMagicVoltage motionRequest;


	private static boolean isAutoDownUsed = false;


	/**
	 * Create ClimberFSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public ClimberFSMSystem() {
		climberMotorLeft = new TalonFX(HardwareMap.CAN_ID_CLIMBER_LEFT);
		climberMotorRight = new TalonFX(HardwareMap.CAN_ID_CLIMBER_RIGHT);
		climberMotorLeft.setControl(new Follower(HardwareMap.CAN_ID_CLIMBER_RIGHT,
			MotorAlignmentValue.Opposed));
		motionRequest = new MotionMagicVoltage(0);
		configureMotor();
		currentState = ClimberFSMState.IDLE;
		groundLimitSwitchLeft = new DigitalInput(HardwareMap.
			CLIMBER_GROUND_LIMIT_SWITCH_DIO_PORT_LEFT);
		groundLimitSwitchRight = new DigitalInput(HardwareMap.
			CLIMBER_GROUND_LIMIT_SWITCH_DIO_PORT_RIGHT);
		reset();
	}

	private void configureMotor() {
		climberMotorRight.setControl(new Follower(
			climberMotorLeft.getDeviceID(), MotorAlignmentValue.Opposed));
		var talonFXConfigs = new TalonFXConfiguration();
		var outputConfigs = talonFXConfigs.MotorOutput;
		outputConfigs.NeutralMode = NeutralModeValue.Brake;

		var swLimitSwitch = talonFXConfigs.SoftwareLimitSwitch;
		swLimitSwitch.ForwardSoftLimitEnable = true;
		swLimitSwitch.ReverseSoftLimitEnable = true;
		swLimitSwitch.ForwardSoftLimitThreshold = ClimberConstants.UPPER_THRESHOLD.in(Inches);
		swLimitSwitch.ReverseSoftLimitThreshold = Inches.of(0).in(Inches);

		var sensorConfig = talonFXConfigs.Feedback;
		sensorConfig.SensorToMechanismRatio = ClimberConstants.ROTS_TO_INCHES;

		var slot0 = talonFXConfigs.Slot0;
		slot0.GravityType = GravityTypeValue.Elevator_Static;
		slot0.kG = ClimberConstants.KG;
		slot0.kS = ClimberConstants.KS;
		slot0.kV = ClimberConstants.KV;
		slot0.kA = ClimberConstants.KA;
		slot0.kP = ClimberConstants.KP;
		slot0.kI = ClimberConstants.KI;
		slot0.kD = ClimberConstants.KD;
		slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

		var motionMagicConfigs = talonFXConfigs.MotionMagic;
		motionMagicConfigs.MotionMagicCruiseVelocity = ClimberConstants.CRUISE_VELO;
		motionMagicConfigs.MotionMagicAcceleration = ClimberConstants.TARGET_ACCEL;
		motionMagicConfigs.MotionMagicExpo_kV = ClimberConstants.EXPO_KV;

		climberMotorLeft.getConfigurator().apply(talonFXConfigs);
		climberMotorRight.getConfigurator().apply(talonFXConfigs);

		climberMotorLeft.setPosition(0);
		climberMotorRight.setPosition(0);

	}
	/**
	 * Get the current FSM state.
	 *
	 * @return current FSM state
	 */
	public ClimberFSMState getCurrentState() {
		return currentState;
	}

	/**
	 * Reset this system to its start state. This may be called from mode init
	 * when the robot is enabled.
	 *
	 * Note this is distinct from the one-time initialization in the constructor
	 * as it may be called multiple times in a boot cycle,
	 * Ex. if the robot is enabled, disabled, then reenabled.
	 */
	public void reset() {
		currentState = ClimberFSMState.IDLE;
		update(null);
	}

	/**
	 * Update FSM based on new inputs. This function only calls the FSM state
	 * specific handlers.
	 *
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *              the robot is in autonomous mode.
	 */
	public void update(TeleopInput input) {
		if (input == null) {
			return;
		}

		Logger.recordOutput("Climber/Current State", currentState.toString());

		switch (currentState) {
			case IDLE -> handleIdleState(input);
			case MANUAL_DIRECT_CONTROL -> handleManualDirectControlState(input);
			case L1_EXTEND -> handleL1ExtendState(input);
			case L1_RETRACT -> handleL1RetractState(input);
			case AUTO_DOWN_1 -> handleL1ExtendState(input);
			case AUTO_DOWN_2 -> handleResetToZero(input);
			case AUTO_UP_1 -> handleL1ExtendState(input);
			case AUTO_UP_2 -> handleL1RetractState(input);
			case LOCKED_FINAL -> handleIdleState(input);
			default -> throw new IllegalStateException("Invalid state: " + currentState.toString());
		}

		currentState = nextState(input);
		updateLogging();
	}

	/**
	 * Update logging values for this system.
	 */
	public void updateLogging() {
		Logger.recordOutput("Climber/Position",
			climberMotorLeft.getPosition().getValueAsDouble());
		Logger.recordOutput("Left motor speed", climberMotorLeft.getDescription());
		Logger.recordOutput("Climber/Velocity",
			climberMotorLeft.getVelocity().getValueAsDouble());
		Logger.recordOutput("Climber/Applied Voltage",
			climberMotorLeft.getMotorVoltage().getValueAsDouble());
		Logger.recordOutput("Climber/Control Request",
			climberMotorLeft.getAppliedControl().toString().
				substring(ClimberConstants.CONTROL_REQUEST_SUBSTRING_START_INDEX));
		Logger.recordOutput("Climber/Height Inches", getClimberHeightInches());
		Logger.recordOutput("Climber/Left Is At Bottom?", groundLimitSwitchLeft.get());
		Logger.recordOutput("Climber/Right Is At Bottom?", groundLimitSwitchRight.get());
		Logger.recordOutput("Climber/Is Extended L1?", getClimberHeightInches()
			>= ClimberConstants.L1_EXTEND_POS.in(Inches)
			- ClimberConstants.POSITION_TOLERANCE_L1.in(Inches));
	}

	private double getClimberHeightInches() {
		return climberMotorLeft.getPosition().getValueAsDouble();
	}

	private boolean isOnGround() {
		double height = getClimberHeightInches();
		return (height <= 0.0 || groundLimitSwitchLeft.get() || groundLimitSwitchRight.get());
	}

	private boolean isExtendedL1() {
		double height = getClimberHeightInches();
		return height >= ClimberConstants.L1_EXTEND_POS.in(Inches)
			- ClimberConstants.POSITION_TOLERANCE_L1.in(Inches);
	}

	private boolean isRetractedL1() {
		double height = getClimberHeightInches();
		return height <= ClimberConstants.L1_RETRACT_POS.in(Inches)
			+ ClimberConstants.POSITION_TOLERANCE_L1.in(Inches);
	}

	private ClimberFSMState nextState(TeleopInput input) {
		if (input == null) {
			return ClimberFSMState.IDLE;
		}

		switch (currentState) {
			case IDLE:
				if (input.isDownButtonPressed() && (isAutoDownUsed)) {
					isAutoDownUsed = true;
					return ClimberFSMState.AUTO_DOWN_1;
				}
				if (input.isClimberManualOverideButtonPressed()) {
					return ClimberFSMState.MANUAL_DIRECT_CONTROL;
				}
				if (input.isClimberNextButtonPressed()) {
					return ClimberFSMState.L1_EXTEND;
				}
				return ClimberFSMState.IDLE;
			case MANUAL_DIRECT_CONTROL:
				if (input.isClimberManualOverideButtonPressed()) {
					return ClimberFSMState.IDLE;
				}
				return ClimberFSMState.IDLE;
			case AUTO_UP_1:
				if (isExtendedL1()) {
					return ClimberFSMState.IDLE;
				}
				return ClimberFSMState.AUTO_UP_1;
			case AUTO_UP_2:
				if (isRetractedL1()) {
					return ClimberFSMState.IDLE;
				}
				return ClimberFSMState.AUTO_UP_2;
			case AUTO_DOWN_1:
				if (input.isDownButtonPressed() && isExtendedL1()) {
					return ClimberFSMState.AUTO_DOWN_2;
				}
				return ClimberFSMState.AUTO_DOWN_1;
			case AUTO_DOWN_2:
				if (isOnGround()) {
					return ClimberFSMState.IDLE;
				}
				return ClimberFSMState.AUTO_DOWN_2;
			case L1_EXTEND:
				if (input.isClimberEmergencyAbortPressed()) {
					return ClimberFSMState.IDLE;
				}
				boolean shouldAdvanceExtendedL1 =
					(input.isClimberNextButtonPressed() && isExtendedL1());
				if (shouldAdvanceExtendedL1) {
					if (currentState == ClimberFSMState.L1_EXTEND) {
						return ClimberFSMState.L1_RETRACT;
					}
				}
				return ClimberFSMState.L1_EXTEND;
			case L1_RETRACT:
			default:
				throw new UnsupportedOperationException("Unknown state");
		}
	}

	private void handleIdleState(TeleopInput input) {
		climberMotorLeft.set(0);
		//currentState = ClimberFSMState.AUTO_UP_1;
	}

	private void handleManualDirectControlState(TeleopInput input) {
		double manualControlValue = MathUtil.applyDeadband(input.getClimberManualControl(),
				ClimberConstants.JOYSTICK_DEADBAND);
		if (groundLimitSwitchLeft.get()) {
			climberMotorLeft.setPosition(0);
		}
		if (groundLimitSwitchRight.get()) {
			climberMotorRight.setPosition(0);
		}
		if (!(groundLimitSwitchLeft.get() && manualControlValue < 0
			&& groundLimitSwitchRight.get())) {
			climberMotorLeft.set(manualControlValue * ClimberConstants.MANUAL_SCALE);
		} else {
			climberMotorLeft.set(0);
		}
	}

	private void handleL1ExtendState(TeleopInput input) {
		if (climberMotorLeft.getMotionMagicAtTarget().getValue()) {
			climberMotorLeft.setControl(motionRequest.withPosition(
				ClimberConstants.L1_EXTEND_POS.in(Inches)
			));
		}
	}

	private void handleL1RetractState(TeleopInput input) {
		if (climberMotorLeft.getMotionMagicAtTarget().getValue()) {
			climberMotorLeft.setControl(motionRequest.withPosition(
				ClimberConstants.L1_RETRACT_POS.in(Inches)
			));
		}
	}

	private void handleResetToZero(TeleopInput input) {
		if (groundLimitSwitchLeft.get()) {
			climberMotorLeft.set(0);
		} else {
			climberMotorLeft.setControl(motionRequest.withPosition(
				ClimberConstants.GROUND.in(Inches)
			));
		}
		if (groundLimitSwitchRight.get()) {
			climberMotorRight.set(0);
		}
	}
}
