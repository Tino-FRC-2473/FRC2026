package frc.robot.systems;

// WPILib Imports
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;


import edu.wpi.first.wpilibj.DigitalInput;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;

// Robot Imports
import frc.robot.constants.IntakeConstants;
import frc.robot.TeleopInput;
import frc.robot.motors.TalonFXWrapper;
import frc.robot.HardwareMap;
import frc.robot.systems.AutoHandlerSystem.AutoFSMState;


public class IntakeFSMSystem {
	public enum IntakeFSMState {
		IDLE_IN_STATE,
		FOLD_OUT_STATE,
		IDLE_OUT_STATE,
		INTAKE_STATE,
		OUTTAKE_STATE,
		FOLD_IN_STATE,
		PARTIAL_OUT_STATE
	}
	/* ======================== Constants ======================== */

	/* ======================== Private variables ======================== */

	private IntakeFSMState currentState;

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
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public IntakeFSMSystem() {

		pivotMotionRequest = new MotionMagicVoltage(0);
		intakeMotionRequest = new MotionMagicVelocityVoltage(0);



		// Perform hardware init using a wrapper class
		// this is so we can see motor outputs during simulatiuons

		//initialize motors
		pivotMotorLeft = new TalonFXWrapper(HardwareMap.CAN_ID_SPARK_PIVOT_LEFT);
		pivotMotorRight = new TalonFXWrapper(HardwareMap.CAN_ID_SPARK_PIVOT_RIGHT);


		intakeMotor = new TalonFXWrapper(HardwareMap.CAN_ID_SPARK_INTAKE);

		var talonFXConfigs = new TalonFXConfiguration();
		var intakeConfigs = new TalonFXConfiguration();

		pivotMotorLeft.setControl(new Follower(pivotMotorRight.getDeviceID(),
			MotorAlignmentValue.Opposed));

		var outputConfigs = talonFXConfigs.MotorOutput;
		outputConfigs.NeutralMode = NeutralModeValue.Brake;

		// apply sw limit
		var swLimitSwitch = talonFXConfigs.SoftwareLimitSwitch;
		swLimitSwitch.ForwardSoftLimitEnable = true; // enable top limit
		swLimitSwitch.ReverseSoftLimitEnable = true; // enable bottom limit
		swLimitSwitch.ForwardSoftLimitThreshold = IntakeConstants.INTAKE_UPPER_TARGET.in(Radians);
		swLimitSwitch.ReverseSoftLimitThreshold = Inches.of(0).in(Inches);

		var sensorConfig = talonFXConfigs.Feedback;
		sensorConfig.SensorToMechanismRatio = IntakeConstants.INTAKE_ROTS_TO_INCHES;

		var slot0Configs = talonFXConfigs.Slot0;
		slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;
		slot0Configs.kG = IntakeConstants.PIVOT_KG;
		slot0Configs.kS = IntakeConstants.PIVOT_KS;
		slot0Configs.kV = IntakeConstants.PIVOT_KV;
		slot0Configs.kA = IntakeConstants.PIVOT_KA;
		slot0Configs.kP = IntakeConstants.PIVOT_KP;
		slot0Configs.kI = IntakeConstants.PIVOT_KI;
		slot0Configs.kD = IntakeConstants.PIVOT_KD;
		slot0Configs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

		var slot1Configs = intakeConfigs.Slot0;
		slot1Configs.kV = IntakeConstants.INTAKE_KV;
		slot1Configs.kA = IntakeConstants.INTAKE_KA;
		slot1Configs.kP = IntakeConstants.INTAKE_KP;
		slot1Configs.kI = IntakeConstants.INTAKE_KI;
		slot1Configs.kD = IntakeConstants.INTAKE_KD;
		slot1Configs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

		var pivotMotionMagicConfigs = talonFXConfigs.MotionMagic;
		pivotMotionMagicConfigs.MotionMagicCruiseVelocity = IntakeConstants.INTAKE_CRUISE_VELO;
		pivotMotionMagicConfigs.MotionMagicAcceleration = IntakeConstants.INTAKE_TARGET_ACCEL;
		pivotMotionMagicConfigs.MotionMagicExpo_kV = IntakeConstants.INTAKE_EXPO_KV;

		var intakeMotionMagicConfigs = intakeConfigs.MotionMagic;
		intakeMotionMagicConfigs.MotionMagicCruiseVelocity = IntakeConstants.INTAKE_CRUISE_VELO;
		intakeMotionMagicConfigs.MotionMagicAcceleration = IntakeConstants.INTAKE_TARGET_ACCEL;
		intakeMotionMagicConfigs.MotionMagicExpo_kV = IntakeConstants.INTAKE_EXPO_KV;


		pivotMotorLeft.getConfigurator().apply(talonFXConfigs);

		BaseStatusSignal.setUpdateFrequencyForAll(
				IntakeConstants.UPDATE_FREQUENCY_HZ,
				pivotMotorLeft.getPosition(),
				pivotMotorLeft.getVelocity(),
				pivotMotorLeft.getAcceleration(),
				pivotMotorLeft.getMotorVoltage(),
				pivotMotorLeft.getRotorPosition(),
				pivotMotorLeft.getRotorVelocity()
		);

		pivotMotorLeft.optimizeBusUtilization();

		pivotMotorRight.getConfigurator().apply(talonFXConfigs);

		BaseStatusSignal.setUpdateFrequencyForAll(
				IntakeConstants.UPDATE_FREQUENCY_HZ,
				pivotMotorRight.getPosition(),
				pivotMotorRight.getVelocity(),
				pivotMotorRight.getAcceleration(),
				pivotMotorRight.getMotorVoltage(),
				pivotMotorRight.getRotorPosition(),
				pivotMotorRight.getRotorVelocity()
		);

		pivotMotorRight.optimizeBusUtilization();

		intakeMotor.getConfigurator().apply(intakeConfigs);

		BaseStatusSignal.setUpdateFrequencyForAll(
				IntakeConstants.UPDATE_FREQUENCY_HZ,
				intakeMotor.getPosition(),
				intakeMotor.getVelocity(),
				intakeMotor.getAcceleration(),
				intakeMotor.getMotorVoltage(),
				intakeMotor.getRotorPosition(),
				intakeMotor.getRotorVelocity()
		);

		intakeMotor.optimizeBusUtilization();


		//initialize limit switch
		groundLimitSwitch = new DigitalInput(HardwareMap.INTAKE_GROUND_LIMIT_SWITCH_DIO_PORT);
		topLimitSwitch = new DigitalInput(HardwareMap.INTAKE_TOP_LIMIT_SWITCH_DIO_PORT);

		// Reset state machine
		pivotMotorLeft.setPosition(0);
		pivotMotorRight.setPosition(0);
		intakeMotor.setPosition(0);

		reset();
	}

	/* ======================== Public methods ======================== */

	// overridden methods don't require javadocs
	// however, you may want to add implementation specific javadocs

	/**
	* Get the current FSM state.
	* @return current FSM state.
	*/
	public IntakeFSMState getCurrentState() {
		return currentState;
	}


	/**
	 * resets the FSM_STATE.
	 */
	public void reset() {
		currentState = IntakeFSMState.IDLE_IN_STATE;

		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}

	/**
	 * updates the current state in IntakeFSMState.
	 * @param input
	 */
	public void update(TeleopInput input) {
		if (input == null) {
			return;
		}
		switch (getCurrentState()) {
			case IDLE_IN_STATE:
				handleIdleInState(input);
				break;

			case FOLD_OUT_STATE:
				handleFoldOutState(input);
				break;

			case IDLE_OUT_STATE:
				handleIdleOutState(input);
				break;

			case INTAKE_STATE:
				handleIntakeState(input);
				break;

			case OUTTAKE_STATE:
				handleOuttakeState(input);
				break;

			case FOLD_IN_STATE:
				handleFoldInState(input);
				break;

			case PARTIAL_OUT_STATE:
				handlePartialOutState(input);
				break;

			default:
				throw new IllegalStateException("Invalid state: " + getCurrentState().toString());
		}
		currentState = nextState(input);
		updateLogging();
	}

	/**
	 * Updates the logging information for the elevator system.
	 */
	public void updateLogging() {
		Logger.recordOutput("Left pivot encoder", pivotMotorLeft.getPosition().getValueAsDouble());
		Logger.recordOutput("Right pivot encoder",
			pivotMotorRight.getPosition().getValueAsDouble());
		Logger.recordOutput("Intake encoder",
			intakeMotor.getPosition().getValueAsDouble());

		Logger.recordOutput("Left pivot velocity", pivotMotorLeft.getVelocity().getValueAsDouble());
		Logger.recordOutput("Right pivot velocity",
			pivotMotorRight.getVelocity().getValueAsDouble());
		Logger.recordOutput("Intake velocity", intakeMotor.getVelocity().getValueAsDouble());

		Logger.recordOutput("Pivot bottom limit switch pressed", isBottomLimitReached());
		Logger.recordOutput("Pivot top limit switch pressed", isTopLimitReached());

		Logger.recordOutput("Intake State", getCurrentState().toString());
		Logger.recordOutput("Left Pivot Motor Voltage",
			pivotMotorLeft.getMotorVoltage().getValueAsDouble());
		Logger.recordOutput("Right Pivot Motor Voltage",
			pivotMotorRight.getMotorVoltage().getValueAsDouble());
		Logger.recordOutput("Intake/Outtake Motor Voltage",
			intakeMotor.getMotorVoltage().getValueAsDouble());

		Logger.recordOutput("Left Pivot Motor Accel",
			pivotMotorLeft.getAcceleration().getValueAsDouble());
		Logger.recordOutput("Right Pivot Motor Accel",
			pivotMotorRight.getAcceleration().getValueAsDouble());

		Logger.recordOutput("LEFT PIVOT ROTR POS",
			pivotMotorLeft.getRotorPosition().getValueAsDouble());
		Logger.recordOutput("LEFT PIVOT ROTR VELO",
			pivotMotorLeft.getRotorVelocity().getValueAsDouble());

		Logger.recordOutput("RIGHT PIVOT ROTR POS",
			pivotMotorRight.getRotorPosition().getValueAsDouble());
		Logger.recordOutput("RIGHT PIVOT ROTR VELO",
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


	/**
	 * Updates autonomous states.
	 * @param autoState
	 * @return state
	 */
	public boolean updateAutonomous(AutoFSMState autoState) {
		switch (autoState) {
			case STATE1:
				return handleAutoState1();
			case STATE2:
				return handleAutoState2();
			case STATE3:
				return handleAutoState3();
			default:
				return true;
		}
	}

	/* ======================== Protected methods ======================== */


	protected IntakeFSMState nextState(TeleopInput input) {
		switch (getCurrentState()) {
			case IDLE_IN_STATE:
				if (input.isFoldOutButtonPressed()) {
					return IntakeFSMState.FOLD_OUT_STATE;
				} else if (input.isPartialOutButtonPressed()) {
					return IntakeFSMState.PARTIAL_OUT_STATE;
				} else {
					return IntakeFSMState.IDLE_IN_STATE;
				}

			case FOLD_OUT_STATE:
				if (isBottomLimitReached()) {
					return IntakeFSMState.IDLE_OUT_STATE;
				} else {
					return IntakeFSMState.FOLD_OUT_STATE;
				}

			case PARTIAL_OUT_STATE:
				if (input.isFoldOutButtonPressed()) {
					return IntakeFSMState.FOLD_OUT_STATE;
				} else if (input.isFoldInButtonPressed()) {
					return IntakeFSMState.FOLD_IN_STATE;
				} else {
					return IntakeFSMState.PARTIAL_OUT_STATE;
				}

			case IDLE_OUT_STATE:
				if (input.isIntakeButtonPressed()) {
					return IntakeFSMState.INTAKE_STATE;
				} else if (input.isOuttakeButtonPressed()) {
					return IntakeFSMState.OUTTAKE_STATE;
				} else if (input.isFoldInButtonPressed()) {
					return IntakeFSMState.FOLD_IN_STATE;
				} else if (input.isPartialOutButtonPressed()) {
					return IntakeFSMState.PARTIAL_OUT_STATE;
				} else {
					return IntakeFSMState.IDLE_OUT_STATE;
				}

			case INTAKE_STATE:
				if (input.isIntakeButtonReleased()) {
					return IntakeFSMState.IDLE_OUT_STATE;
				} else {
					return IntakeFSMState.INTAKE_STATE;
				}

			case OUTTAKE_STATE:
				if (input.isOuttakeButtonReleased()) {
					return IntakeFSMState.IDLE_OUT_STATE;
				} else {
					return IntakeFSMState.OUTTAKE_STATE;
				}

			case FOLD_IN_STATE:
				if (isTopLimitReached()) {
					return IntakeFSMState.IDLE_IN_STATE;
				} else {
					return IntakeFSMState.FOLD_IN_STATE;
				}

			default:
				throw new IllegalStateException("Invalid state: " + getCurrentState().toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in IDLE_IN_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleIdleInState(TeleopInput input) {
	}
	/**
	 * Handle behavior in START_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleFoldOutState(TeleopInput input) {
		pivotMotorRight.setControl(pivotMotionRequest.
			withPosition(IntakeConstants.INTAKE_GROUND_TARGET));
	}
	/**
	 * Handle behavior in PARTIAL_OUT_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handlePartialOutState(TeleopInput input) {
		pivotMotorRight.setControl(pivotMotionRequest.
			withPosition(IntakeConstants.PARTIAL_OUT_POSITION));
	}
	/**
	 * Handle behavior in START_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleIdleOutState(TeleopInput input) {
	}
	/**
	 * Handle behavior in START_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleIntakeState(TeleopInput input) {
		intakeMotor.setControl(intakeMotionRequest.
			withVelocity(IntakeConstants.INTAKE_TARGET_VELOCITY));
	}
	/**
	 * Handle behavior in START_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleOuttakeState(TeleopInput input) {
		intakeMotor.setControl(intakeMotionRequest.
			withVelocity(IntakeConstants.OUTTAKE_TARGET_VELOCITY));
	}
	/**
	 * Handle behavior in OTHER_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleFoldInState(TeleopInput input) {
		pivotMotorRight.setControl(pivotMotionRequest.
			withPosition(IntakeConstants.INTAKE_UPPER_TARGET));
	}

	/**
	 * Performs action for auto STATE1.
	 * @return if the action carried out has finished executing
	 */
	private boolean handleAutoState1() {
		return true;
	}

	/**
	 * Performs action for auto STATE2.
	 * @return if the action carried out has finished executing
	 */
	private boolean handleAutoState2() {
		return true;
	}

	/**
	 * |
	 * Performs action for auto STATE3.
	 * @return if the action carried out has finished executing
	 */
	private boolean handleAutoState3() {
		return true;
	}

	/* ======================== Private methods ======================== */
	/**
	 * Getter for the result of the elevator's bottom limit switch.
	 * @return whether the limit is reached
	 */
	private boolean isBottomLimitReached() {
		return groundLimitSwitch.get(); // switch is normally open
	}

	private boolean isTopLimitReached() {
		return topLimitSwitch.get(); // switch is normally open
	}

}
