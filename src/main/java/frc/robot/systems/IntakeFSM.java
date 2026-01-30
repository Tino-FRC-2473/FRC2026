package frc.robot.systems;

// WPILib units
import static edu.wpi.first.units.Units.Radians;

// Library imports
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.wpilibj.DigitalInput;

// Robot imports
import frc.robot.Constants.IntakeConstants;
import frc.robot.HardwareMap;
import frc.robot.input.Input;
import frc.robot.input.InputTypes.ButtonInput;
import frc.robot.motors.TalonFXWrapper;

public class IntakeFSM extends FSMSystem<IntakeFSM.IntakeState> {

	/* ======================== FSM state enum ======================== */

	public enum IntakeState {
		IDLE_IN,
		IDLE_OUT,
		PARTIAL_OUT,
		FOLDING_IN,
		FOLDING_OUT,
		INTAKING,
		OUTTAKING
	}

	/* ======================== Private variables ======================== */

	// MotionMagic requests
	private MotionMagicVoltage pivotMotionRequest;
	private MotionMagicVelocityVoltage intakeMotionRequest;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private TalonFXWrapper pivotMotorLeft;
	private TalonFXWrapper pivotMotorRight;
	private TalonFXWrapper intakeMotor;

	private DigitalInput groundLimitSwitch;
	private DigitalInput topLimitSwitch;

	/* ======================== Constructor ======================== */

	/**
	 * Create the Intake FSM system.
	 */
	public IntakeFSM() {
		// Hardware initialization
		initializePivotMotors();
		initializeIntakeMotor();
		initializeLimitSwitches();

		// Reset state machine
		reset();
	}

	/* ======================== Public methods ======================== */

	/**
	 * Get the status of the ground limit switch.
	 * @return whether the limit is reached
	 */
	private boolean isGroundLimitReached() {
		return groundLimitSwitch.get();
	}

	/**
	 * Get the status of the top limit switch.
	 * @return whether the limit is reached
	 */
	private boolean isTopLimitReached() {
		return topLimitSwitch.get();
	}

	/**
	 * Reset current the FSM state.
	 */
	public void reset() {
		setCurrentState(IntakeState.IDLE_IN);

		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}

	@Override
	public void update(Input input) {
		if (input == null) {
			return;
		}

		switch (getCurrentState()) {
			case IDLE_IN:
				handleIdleInState(input);
				break;

			case IDLE_OUT:
				handleIdleOutState(input);
				break;

			case PARTIAL_OUT:
				handlePartialOutState(input);
				break;

			case INTAKING:
				handleIntakingState(input);
				break;

			case OUTTAKING:
				handleOuttakingState(input);
				break;

			case FOLDING_OUT:
				handleFoldingOutState(input);
				break;

			case FOLDING_IN:
				handleFoldingInState(input);
				break;

			default:
				throw new IllegalStateException("Cannot update an invalid current state: "
						+ getCurrentState().toString());
		}

		setCurrentState(nextState(input));
		updateLogging();
	}

	/* ======================== Protected methods ======================== */

	@Override
	protected IntakeState nextState(Input input) {
		switch (getCurrentState()) {
			case IDLE_IN:
				if (input.getButtonPressed(ButtonInput.INTAKE_FOLD_OUT)) {
					return IntakeState.FOLDING_OUT;
				} else if (input.getButtonPressed(ButtonInput.INTAKE_PARTIAL_OUT)) {
					return IntakeState.PARTIAL_OUT;
				} else {
					return IntakeState.IDLE_IN;
				}

			case IDLE_OUT:
				if (input.getButtonPressed(ButtonInput.INTAKE_INTAKE)) {
					return IntakeState.INTAKING;
				} else if (input.getButtonPressed(ButtonInput.INTAKE_OUTTAKE)) {
					return IntakeState.OUTTAKING;
				} else if (input.getButtonPressed(ButtonInput.INTAKE_FOLD_IN)) {
					return IntakeState.FOLDING_IN;
				} else if (input.getButtonPressed(ButtonInput.INTAKE_PARTIAL_OUT)) {
					return IntakeState.PARTIAL_OUT;
				} else {
					return IntakeState.IDLE_OUT;
				}

			case PARTIAL_OUT:
				if (input.getButtonPressed(ButtonInput.INTAKE_FOLD_OUT)) {
					return IntakeState.FOLDING_OUT;
				} else if (input.getButtonPressed(ButtonInput.INTAKE_FOLD_IN)) {
					return IntakeState.FOLDING_IN;
				} else {
					return IntakeState.PARTIAL_OUT;
				}

			case FOLDING_IN:
				if (isTopLimitReached()) {
					return IntakeState.IDLE_IN;
				} else {
					return IntakeState.FOLDING_IN;
				}

			case FOLDING_OUT:
				if (isGroundLimitReached()) {
					return IntakeState.IDLE_OUT;
				} else {
					return IntakeState.FOLDING_OUT;
				}

			case INTAKING:
				if (input.getButtonReleased(ButtonInput.INTAKE_INTAKE)) {
					return IntakeState.IDLE_OUT;
				} else {
					return IntakeState.INTAKING;
				}

			case OUTTAKING:
				if (input.getButtonReleased(ButtonInput.INTAKE_OUTTAKE)) {
					return IntakeState.IDLE_OUT;
				} else {
					return IntakeState.OUTTAKING;
				}

			default:
				throw new IllegalStateException("Invalid state: " + getCurrentState().toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */

	/**
	 * Handle idle in state.
	 * @param input the input
	 */
	private void handleIdleInState(Input input) {
	}

	/**
	 * Handle idle out state.
	 * @param input the input
	 */
	private void handleIdleOutState(Input input) {
	}

	/**
	 * Handle partial out state.
	 * @param input the input
	 */
	private void handlePartialOutState(Input input) {
		pivotMotorRight.setControl(pivotMotionRequest
				.withPosition(IntakeConstants.PARTIAL_OUT_TARGET_ANGLE));
	}

	/**
	 * Handle folding in state.
	 * @param input the input
	 */
	private void handleFoldingInState(Input input) {
		pivotMotorRight.setControl(pivotMotionRequest
				.withPosition(IntakeConstants.UPPER_TARGET_ANGLE));
	}

	/**
	 * Handle folding out state.
	 * @param input the input
	 */
	private void handleFoldingOutState(Input input) {
		pivotMotorRight.setControl(pivotMotionRequest
				.withPosition(IntakeConstants.GROUND_TARGET_ANGLE));
	}

	/**
	 * Handle intaking state.
	 * @param input the input
	 */
	private void handleIntakingState(Input input) {
		intakeMotor.setControl(intakeMotionRequest
				.withVelocity(IntakeConstants.INTAKE_TARGET_VELOCITY));
	}

	/**
	 * Handle outtaking state.
	 * @param input the input
	 */
	private void handleOuttakingState(Input input) {
		intakeMotor.setControl(intakeMotionRequest
				.withVelocity(IntakeConstants.OUTTAKE_TARGET_VELOCITY));
	}

	/* ======================== Private methods ======================== */

	private void initializePivotMotors() {
		// Initialize MotionMagic request
		pivotMotionRequest = new MotionMagicVoltage(0);

		// Initialize motor configuration
		TalonFXConfiguration config = initializePivotMotorConfiguration();

		// Initialize pivot motors
		initializeLeftPivotMotor(config);
		initializeRightPivotMotor(config);

		// Set right motor to follow left
		pivotMotorLeft.setControl(new Follower(
			pivotMotorRight.getDeviceID(),
			MotorAlignmentValue.Opposed
		));
	}

	private TalonFXConfiguration initializePivotMotorConfiguration() {
		TalonFXConfiguration config = new TalonFXConfiguration();

		SoftwareLimitSwitchConfigs limitSwitchConfig = config.SoftwareLimitSwitch;
		limitSwitchConfig.ForwardSoftLimitEnable = true;
		limitSwitchConfig.ReverseSoftLimitEnable = true;
		limitSwitchConfig.ForwardSoftLimitThreshold = IntakeConstants.UPPER_TARGET_ANGLE
				.in(Radians);
		limitSwitchConfig.ReverseSoftLimitThreshold = IntakeConstants.GROUND_TARGET_ANGLE
				.in(Radians);

		FeedbackConfigs feedbackConfig = config.Feedback;
		feedbackConfig.SensorToMechanismRatio = IntakeConstants.PIVOT_GEARING;

		Slot0Configs slot0Config = config.Slot0;
		slot0Config.GravityType = GravityTypeValue.Arm_Cosine;
		slot0Config.kG = IntakeConstants.PIVOT_G;
		slot0Config.kS = IntakeConstants.PIVOT_S;
		slot0Config.kV = IntakeConstants.PIVOT_V;
		slot0Config.kA = IntakeConstants.PIVOT_A;
		slot0Config.kP = IntakeConstants.PIVOT_P;
		slot0Config.kI = IntakeConstants.PIVOT_I;
		slot0Config.kD = IntakeConstants.PIVOT_D;
		slot0Config.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

		MotionMagicConfigs pivotMotionMagicConfigs = config.MotionMagic;
		pivotMotionMagicConfigs.MotionMagicCruiseVelocity = IntakeConstants.PIVOT_CRUISE_VELOCITY;
		pivotMotionMagicConfigs.MotionMagicAcceleration = IntakeConstants.PIVOT_ACCELERATION;
		pivotMotionMagicConfigs.MotionMagicExpo_kV = IntakeConstants.PIVOT_EXPO_KV;

		return config;
	}

	private void initializeLeftPivotMotor(TalonFXConfiguration config) {
		// Initialize motor and configuration
		pivotMotorLeft = new TalonFXWrapper(HardwareMap.CAN_ID_SPARK_PIVOT_LEFT);
		pivotMotorLeft.getConfigurator().apply(config);

		BaseStatusSignal.setUpdateFrequencyForAll(
				IntakeConstants.UPDATE_FREQUENCY,
				pivotMotorLeft.getPosition(),
				pivotMotorLeft.getVelocity(),
				pivotMotorLeft.getAcceleration(),
				pivotMotorLeft.getMotorVoltage(),
				pivotMotorLeft.getRotorPosition(),
				pivotMotorLeft.getRotorVelocity()
		);
		pivotMotorLeft.optimizeBusUtilization();

		// Clear motor position
		pivotMotorLeft.setPosition(0);
	}

	private void initializeRightPivotMotor(TalonFXConfiguration config) {
		// Initialize motor and configuration
		pivotMotorRight = new TalonFXWrapper(HardwareMap.CAN_ID_SPARK_PIVOT_RIGHT);
		pivotMotorRight.getConfigurator().apply(config);

		BaseStatusSignal.setUpdateFrequencyForAll(
				IntakeConstants.UPDATE_FREQUENCY,
				pivotMotorRight.getPosition(),
				pivotMotorRight.getVelocity(),
				pivotMotorRight.getAcceleration(),
				pivotMotorRight.getMotorVoltage(),
				pivotMotorRight.getRotorPosition(),
				pivotMotorRight.getRotorVelocity()
		);
		pivotMotorRight.optimizeBusUtilization();

		// Clear motor position
		pivotMotorRight.setPosition(0);
	}

	private void initializeIntakeMotor() {
		// Initialize MotionMagic request
		intakeMotionRequest = new MotionMagicVelocityVoltage(0);

		// Initialize motor configuration
		TalonFXConfiguration config = initializeIntakeMotorConfiguration();

		// Initialize intake motor
		intakeMotor = new TalonFXWrapper(HardwareMap.CAN_ID_SPARK_INTAKE);
		intakeMotor.getConfigurator().apply(config);

		BaseStatusSignal.setUpdateFrequencyForAll(
				IntakeConstants.UPDATE_FREQUENCY,
				intakeMotor.getPosition(),
				intakeMotor.getVelocity(),
				intakeMotor.getAcceleration(),
				intakeMotor.getMotorVoltage(),
				intakeMotor.getRotorPosition(),
				intakeMotor.getRotorVelocity()
		);
		intakeMotor.optimizeBusUtilization();

		// Clear motor position
		intakeMotor.setPosition(0);
	}

	private TalonFXConfiguration initializeIntakeMotorConfiguration() {
		TalonFXConfiguration config = new TalonFXConfiguration();

		FeedbackConfigs feedbackConfig = config.Feedback;
		feedbackConfig.SensorToMechanismRatio = IntakeConstants.INTAKE_GEARING;

		Slot0Configs slot0Config = config.Slot0;
		slot0Config.kV = IntakeConstants.INTAKE_V;
		slot0Config.kA = IntakeConstants.INTAKE_A;
		slot0Config.kP = IntakeConstants.INTAKE_P;
		slot0Config.kI = IntakeConstants.INTAKE_I;
		slot0Config.kD = IntakeConstants.INTAKE_D;
		slot0Config.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

		MotionMagicConfigs motionMagicConfig = config.MotionMagic;
		motionMagicConfig.MotionMagicCruiseVelocity = IntakeConstants.INTAKE_CRUISE_VELOCITY;
		motionMagicConfig.MotionMagicAcceleration = IntakeConstants.INTAKE_ACCELERATION;
		motionMagicConfig.MotionMagicExpo_kV = IntakeConstants.INTAKE_EXPO_KV;

		return config;
	}

	private void initializeLimitSwitches() {
		groundLimitSwitch = new DigitalInput(HardwareMap.INTAKE_GROUND_LIMIT_SWITCH_DIO_PORT);
		topLimitSwitch = new DigitalInput(HardwareMap.INTAKE_TOP_LIMIT_SWITCH_DIO_PORT);
	}

	private void updateLogging() {
		Logger.recordOutput("Intake/Left pivot encoder",
			pivotMotorLeft.getPosition().getValueAsDouble());
		Logger.recordOutput("Intake/Right pivot encoder",
			pivotMotorRight.getPosition().getValueAsDouble());
		Logger.recordOutput("Intake encoder",
			intakeMotor.getPosition().getValueAsDouble());

		Logger.recordOutput("Intake/Left pivot velocity",
			pivotMotorLeft.getVelocity().getValueAsDouble());
		Logger.recordOutput("Intake/Right pivot velocity",
			pivotMotorRight.getVelocity().getValueAsDouble());
		Logger.recordOutput("Intake velocity", intakeMotor.getVelocity().getValueAsDouble());

		Logger.recordOutput("Intake/Pivot bottom limit switch pressed", isGroundLimitReached());
		Logger.recordOutput("Intake/Pivot top limit switch pressed", isTopLimitReached());

		Logger.recordOutput("Intake State", getCurrentState().toString());
		Logger.recordOutput("Intake/Left Pivot Motor Voltage",
			pivotMotorLeft.getMotorVoltage().getValueAsDouble());
		Logger.recordOutput("Intake/Right Pivot Motor Voltage",
			pivotMotorRight.getMotorVoltage().getValueAsDouble());
		Logger.recordOutput("Intake/Outtake Motor Voltage",
			intakeMotor.getMotorVoltage().getValueAsDouble());

		Logger.recordOutput("Intake/LEFT PIVOT ROTR POS",
			pivotMotorLeft.getRotorPosition().getValueAsDouble());
		Logger.recordOutput("Intake/LEFT PIVOT ROTR VELO",
			pivotMotorLeft.getRotorVelocity().getValueAsDouble());

		Logger.recordOutput("Intake/RIGHT PIVOT ROTR POS",
			pivotMotorRight.getRotorPosition().getValueAsDouble());
		Logger.recordOutput("Intake/RIGHT PIVOT ROTR VELO",
			pivotMotorRight.getRotorVelocity().getValueAsDouble());

		Logger.recordOutput("INTAKE ROTR POS",
			intakeMotor.getRotorPosition().getValueAsDouble());
		Logger.recordOutput("INTAKE ROTR VELO",
			intakeMotor.getRotorVelocity().getValueAsDouble());

		/*telemetry and logging: currently not necessary
		MechLogging.getInstance().updateElevatorPose3d(Angle.ofBaseUnits(
			elevatorSim.getPositionMeters(), Radians
		));
		*/
	}

}
