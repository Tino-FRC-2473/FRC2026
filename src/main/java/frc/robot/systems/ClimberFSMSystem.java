package frc.robot.systems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.HardwareMap;
import frc.robot.TeleopInput;

import org.littletonrobotics.junction.Logger;

import frc.robot.constants.Constants;

import com.ctre.phoenix6.controls.MotionMagicVoltage;

import static edu.wpi.first.units.Units.Inches;

import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class ClimberFSMSystem {
	public enum ClimberFSMState {
		IDLE,
		MANUAL_DIRECT_CONTROL,
		L1_EXTEND,
		L1_RETRACT,
		L2_EXTEND,
		L2_RETRACT,
		L3_EXTEND,
		L3_RETRACT,
		AUTO_DOWN,
		AUTO_UP,
		AUTO_IDLE,
		LOCKED_FINAL
	}

	private final TalonFX climberMotor;
	private final TalonFX climberTiltMotor;
	private final DigitalInput groundLimitSwitch;

	private ClimberFSMState currentState;
	private MotionMagicVoltage motionRequest;

	// TODO: Update these positions based on actual climber design
	// private static final Distance EXAMPLE_POS = Inches.of(100.0);
	private static final double L1_EXTEND_POS = 100.0;
	private static final double L1_RETRACT_POS = 0.0;
	private static final double L2_L3_EXTEND_POS = 150.0;
	private static final double L2_L3_RETRACT_POS = 0.0;
	private static final Distance targetPosition = Inches.of(0.0);

	private boolean firstStage = true;

	/**
	 * Create ClimberFSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public ClimberFSMSystem() {
		climberMotor = new TalonFX(HardwareMap.CAN_ID_CLIMBER);
		climberTiltMotor = new TalonFX(HardwareMap.CAN_ID_CLIMBER_TILT);

		motionRequest = new MotionMagicVoltage(0);
		configureMotor();
		groundLimitSwitch = new DigitalInput(HardwareMap.CLIMBER_GROUND_LIMIT_SWITCH_DIO_PORT);
		reset();
	}

	private void configureMotor() {
		var talonFXConfigs = new TalonFXConfiguration();
		var outputConfigs = talonFXConfigs.MotorOutput;
		outputConfigs.NeutralMode = NeutralModeValue.Brake;

		var swLimitSwitch = talonFXConfigs.SoftwareLimitSwitch;
		swLimitSwitch.ForwardSoftLimitEnable = true;
		swLimitSwitch.ReverseSoftLimitEnable = true;
		swLimitSwitch.ForwardSoftLimitThreshold = Constants.CLIMBER_UPPER_THRESHOLD.in(Inches);
		swLimitSwitch.ReverseSoftLimitThreshold = Inches.of(0).in(Inches);

		var sensorConfig = talonFXConfigs.Feedback;
		sensorConfig.SensorToMechanismRatio = Constants.CLIMBER_ROTS_TO_INCHES;

		var slot0 = talonFXConfigs.Slot0;
		slot0.GravityType = GravityTypeValue.Elevator_Static;
		// TODO: Verify this is the correct gravity compensation type
		slot0.kG = Constants.CLIMBER_KG;
		slot0.kS = Constants.CLIMBER_KS;
		slot0.kV = Constants.CLIMBER_KV;
		slot0.kA = Constants.CLIMBER_KA;
		slot0.kP = Constants.CLIMBER_KP;
		slot0.kI = Constants.CLIMBER_KI;
		slot0.kD = Constants.CLIMBER_KD;
		slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

		var motionMagicConfigs = talonFXConfigs.MotionMagic;
		motionMagicConfigs.MotionMagicCruiseVelocity = Constants.CLIMBER_CRUISE_VELO;
		motionMagicConfigs.MotionMagicAcceleration = Constants.CLIMBER_TARGET_ACCEL;
		motionMagicConfigs.MotionMagicExpo_kV = Constants.CLIMBER_EXPO_KV;

		climberMotor.getConfigurator().apply(talonFXConfigs);

		climberMotor.setPosition(0);

		var tiltConfigs = new TalonFXConfiguration();
		var tiltOutputConfigs = tiltConfigs.MotorOutput;
		tiltOutputConfigs.NeutralMode = NeutralModeValue.Brake;

		var sensorTiltconfig = tiltConfigs.Feedback;
		sensorTiltconfig.SensorToMechanismRatio = Constants.CLIMBER_TILT_ROTS_TO_ANGLE;

		var slot0tilt = tiltConfigs.Slot0;
		// TODO: Verify this is the correct gravity compensation type
		slot0tilt.GravityType = GravityTypeValue.Arm_Cosine;
		slot0tilt.kG = Constants.CLIMBER_TILT_KG;
		slot0tilt.kS = Constants.CLIMBER_TILT_KS;
		slot0tilt.kV = Constants.CLIMBER_TILT_KV;
		slot0tilt.kA = Constants.CLIMBER_TILT_KA;
		slot0tilt.kP = Constants.CLIMBER_TILT_KP;
		slot0tilt.kI = Constants.CLIMBER_TILT_KI;
		slot0tilt.kD = Constants.CLIMBER_TILT_KD;
		slot0tilt.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

		var motionMagicConfigstilt = tiltConfigs.MotionMagic;
		motionMagicConfigstilt.MotionMagicCruiseVelocity = Constants.CLIMBER_TILT_CRUISE_VELO;
		motionMagicConfigstilt.MotionMagicAcceleration = Constants.CLIMBER_TILT_TARGET_ACCEL;
		motionMagicConfigstilt.MotionMagicExpo_kV = Constants.CLIMBER_TILT_EXPO_KV;

		climberTiltMotor.getConfigurator().apply(tiltConfigs);

		climberMotor.setPosition(0);
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

		switch (currentState) {
			case IDLE -> handleIdleState(input);
			case MANUAL_DIRECT_CONTROL -> handleManualDirectControlState(input);
			case L1_EXTEND -> handleL1ExtendState(input);
			case L1_RETRACT -> handleL1RetractState(input);
			case L2_EXTEND -> handleL2ExtendState(input);
			case L2_RETRACT -> handleL2RetractState(input);
			case L3_EXTEND -> handleL3ExtendState(input);
			case L3_RETRACT -> handleL3RetractState(input);
			case AUTO_DOWN -> handleAutoDownState(input);
			case AUTO_UP -> handleAutoUpState(input);
			case AUTO_IDLE -> handleIdleState(input);
			case LOCKED_FINAL -> handleIdleState(input);
			default -> throw new IllegalStateException("Invalid state: " + currentState.toString());
		}

		currentState = nextState(input);
	}

	public void updateLogging() {
		Logger.recordOutput("Climber encoder absolute",
			climberMotor.getPosition().getValueAsDouble());
		Logger.recordOutput("Climber encoder relative",
			climberMotor.getPosition().getValueAsDouble());
		Logger.recordOutput("Climber encoder degrees",
			climberTiltMotor.getPosition().getValueAsDouble());
		Logger.recordOutput("Climber velocity", climberMotor.getVelocity().getValueAsDouble());
		Logger.recordOutput("Climber applied voltage",
			climberMotor.getMotorVoltage().getValueAsDouble());
		Logger.recordOutput("Climber state", currentState.toString());
		Logger.recordOutput("Climber control request", climberMotor.getAppliedControl().toString());
		Logger.recordOutput("Climber switch pressed?", isGroundLimitSwitchPressed());
		Logger.recordOutput("Climber target position", targetPosition);
		Logger.recordOutput("Climber height inches", getClimberHeightInches());
		Logger.recordOutput("Climber is latched?", isLatched());
		Logger.recordOutput("Climber is on ground?", isOnGround());
		Logger.recordOutput("Climber is extended L1?", isExtendedL1());
		Logger.recordOutput("Climber is extended L2 or L3?", isExtendedL2OrL3());
		Logger.recordOutput("Climber first stage?", firstStage);
	}

	private double getClimberHeightInches() {
		return climberMotor.getPosition().getValueAsDouble();
	}

	private boolean isGroundLimitSwitchPressed() {
		return groundLimitSwitch.get();
	}
	private boolean isLatched() {
		// TODO Auto-generated method stub
		return false;
	}

	private boolean isOnGround() {
		return groundLimitSwitch.get();
	}

	private boolean isExtendedL1() {
		double height = getClimberHeightInches();
		return height >= L1_EXTEND_POS - Constants.CLIMBER_POSITION_TOLERANCE_L1;
	}

	private boolean isExtendedL2OrL3() {
		double height = getClimberHeightInches();
		return height >= L2_L3_EXTEND_POS - Constants.CLIMBER_POSITION_TOLERANCE_L2_L3;
	}

	private ClimberFSMState nextState(TeleopInput input) {
		if (input == null) {
			return ClimberFSMState.IDLE;
		}

		switch (currentState) {
			case IDLE:
				if (input.isManualOverideButtonPressed()) {
					return ClimberFSMState.MANUAL_DIRECT_CONTROL;
				}
				if (input.isNextButtonPressed()) {
					return ClimberFSMState.L1_EXTEND;
				}
				return ClimberFSMState.IDLE;
			case MANUAL_DIRECT_CONTROL:
				if (input.isManualOverideButtonPressed()) {
					return ClimberFSMState.IDLE;
				}
				return ClimberFSMState.IDLE;
			case AUTO_UP:
				if (isLatched()) {
					return ClimberFSMState.AUTO_IDLE;
				}
				return ClimberFSMState.AUTO_UP;
			case AUTO_IDLE:
				if (!DriverStation.isAutonomous()) {
					return ClimberFSMState.AUTO_DOWN;
				}
				return ClimberFSMState.AUTO_IDLE;
			case AUTO_DOWN:
				if (isOnGround()) {
					return ClimberFSMState.IDLE;
				}
				return ClimberFSMState.AUTO_DOWN;
			case L3_RETRACT:
				if (input.isEmergencyAbortPressed()) {
					return ClimberFSMState.IDLE;
				}
				if (isLatched()) {
					return ClimberFSMState.LOCKED_FINAL;
				}
				return ClimberFSMState.L3_RETRACT;
			case L1_EXTEND:
				if (input.isEmergencyAbortPressed()) {
					return ClimberFSMState.IDLE;
				}
				boolean shouldAdvanceExtendedL1 = (input.isNextButtonPressed() && isExtendedL1());
				if (shouldAdvanceExtendedL1) {
					if (currentState == ClimberFSMState.L1_EXTEND) {
						return ClimberFSMState.L1_RETRACT;
					}
				}
				return ClimberFSMState.L1_EXTEND;
			case L1_RETRACT:
			case L2_EXTEND:
			case L2_RETRACT:
			case L3_EXTEND:
				if (input.isEmergencyAbortPressed()) {
					return ClimberFSMState.IDLE;
				}

				boolean shouldAdvanceLatched = (input.isNextButtonPressed() && isLatched());
				if (shouldAdvanceLatched) {
					if (currentState == ClimberFSMState.L1_RETRACT) {
						return ClimberFSMState.L2_EXTEND;
					} else if (currentState == ClimberFSMState.L2_RETRACT) {
						return ClimberFSMState.L3_EXTEND;
					}
				}

				boolean shouldAdvanceExtendedL2OrL3 = (input.isNextButtonPressed()
						&& isExtendedL2OrL3());
				if (shouldAdvanceExtendedL2OrL3) {
					if (currentState == ClimberFSMState.L2_EXTEND) {
						return ClimberFSMState.L2_RETRACT;
					} else if (currentState == ClimberFSMState.L3_EXTEND) {
						return ClimberFSMState.L3_RETRACT;
					}
				}
				return getCurrentState();
			default:
				throw new UnsupportedOperationException("Unknown state");
		}
	}

	private void handleIdleState(TeleopInput input) {
		climberMotor.set(0);
		climberTiltMotor.set(0);
	}

	private void handleManualDirectControlState(TeleopInput input) {
		double manualControlValue = MathUtil.applyDeadband(input.getClimberManualControl(),
				Constants.CLIMBER_JOYSTICK_DEADBAND);
		if (isOnGround()) {
			climberMotor.setPosition(0);
		}
		if (!(isOnGround() && manualControlValue < 0)) {
			climberMotor.set(manualControlValue * Constants.CLIMBER_MANUAL_SCALE);
		} else {
			climberMotor.set(0);
		}
	}

	private void handleL1ExtendState(TeleopInput input) {
		climberTiltMotor.setControl(motionRequest.withPosition(Constants.CLIMBER_TILT_EXTEND_POS));
		if (climberMotor.getMotionMagicAtTarget().getValue()) {
			climberMotor.setControl(motionRequest.withPosition(L1_EXTEND_POS));
		}

	}

	private void handleL1RetractState(TeleopInput input) {
		climberTiltMotor.setControl(motionRequest.withPosition(Constants.CLIMBER_TILT_RETRACT_POS));
		if (climberMotor.getMotionMagicAtTarget().getValue()) {
			climberMotor.setControl(motionRequest.withPosition(L1_RETRACT_POS));
		}
	}

	private void handleL2ExtendState(TeleopInput input) {
		climberTiltMotor.setControl(motionRequest.withPosition(Constants.CLIMBER_TILT_EXTEND_POS));
		if (climberMotor.getMotionMagicAtTarget().getValue()) {
			climberMotor.setControl(motionRequest.withPosition(L2_L3_EXTEND_POS));
		}
	}

	private void handleL2RetractState(TeleopInput input) {
		climberTiltMotor.setControl(motionRequest.withPosition(Constants.CLIMBER_TILT_RETRACT_POS));
		if (climberMotor.getMotionMagicAtTarget().getValue()) {
			climberMotor.setControl(motionRequest.withPosition(L2_L3_RETRACT_POS));
		}
	}

	private void handleL3ExtendState(TeleopInput input) {
		climberTiltMotor.setControl(motionRequest.withPosition(Constants.CLIMBER_TILT_EXTEND_POS));
		if (climberMotor.getMotionMagicAtTarget().getValue()) {
			climberMotor.setControl(motionRequest.withPosition(L2_L3_EXTEND_POS));
		}
	}

	private void handleL3RetractState(TeleopInput input) {
		climberTiltMotor.setControl(motionRequest.withPosition(Constants.CLIMBER_TILT_RETRACT_POS));
		if (climberMotor.getMotionMagicAtTarget().getValue()) {
			climberMotor.setControl(motionRequest.withPosition(L2_L3_RETRACT_POS));
		}
	}

	private void handleAutoDownState(TeleopInput input) {
		climberMotor.setControl(motionRequest.withPosition(L1_EXTEND_POS));
		if (climberMotor.getMotionMagicAtTarget().getValue()) {
			climberTiltMotor.setControl(motionRequest
					.withPosition(Constants.CLIMBER_TILT_EXTEND_POS));
		}





		if (Boolean.TRUE.equals(climberMotor.getMotionMagicAtTarget().getValue())) {
			climberTiltMotor.setControl(motionRequest
					.withPosition(Constants.CLIMBER_TILT_RETRACT_POS));
		}
		if (Boolean.TRUE.equals(climberMotor.getMotionMagicAtTarget().getValue()
		// TODO: fix
				&& Boolean.TRUE.equals(climberTiltMotor.getMotionMagicAtTarget().getValue()))) {
			climberMotor.setControl(motionRequest.withPosition(0));
			climberTiltMotor.setControl(motionRequest.withPosition(0));
		}
	}

	private void handleAutoUpState(TeleopInput input) {
		if (firstStage) {
			climberTiltMotor.setControl(motionRequest
					.withPosition(Constants.CLIMBER_TILT_EXTEND_POS));
			if (Boolean.TRUE.equals(climberTiltMotor.getMotionMagicAtTarget().getValue())) {
				climberMotor.setControl(motionRequest.withPosition(L1_EXTEND_POS));
				if (Boolean.TRUE.equals(climberMotor.getMotionMagicAtTarget().getValue())) {
					firstStage = false;
				}
			}
		}
		if (!firstStage) {
			climberTiltMotor.setControl(motionRequest
					.withPosition(Constants.CLIMBER_TILT_RETRACT_POS));
			if (Boolean.TRUE.equals(climberMotor.getMotionMagicAtTarget().getValue())) {
				climberMotor.setControl(motionRequest.withPosition(L1_RETRACT_POS));
			}
		}

	}

	private void handleAutoIdleState(TeleopInput input) {
		climberTiltMotor.set(0);
		climberMotor.set(0);
	}

	private void handleLockedFinalState(TeleopInput input) {
		climberTiltMotor.set(0);
		climberMotor.set(0);
	}

}
