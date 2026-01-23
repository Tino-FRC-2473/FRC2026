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
import frc.robot.constants.Constants;
import frc.robot.TeleopInput;
import frc.robot.motors.TalonFXWrapper;
import frc.robot.HardwareMap;
import frc.robot.systems.AutoHandlerSystem.AutoFSMState;

enum FSMState {
	IDLE_IN_STATE,
	FOLD_OUT_STATE,
	IDLE_OUT_STATE,
	INTAKE_STATE,
	OUTTAKE_STATE,
	FOLD_IN_STATE,
	PARTIAL_OUT_STATE
}

public class IntakeFSMSystem extends FSMSystem<FSMState> {
	/* ======================== Constants ======================== */

	/* ======================== Private variables ======================== */

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
		swLimitSwitch.ForwardSoftLimitThreshold = Constants.INTAKE_UPPER_TARGET.in(Radians);
		swLimitSwitch.ReverseSoftLimitThreshold = Inches.of(0).in(Inches);

		var sensorConfig = talonFXConfigs.Feedback;
		sensorConfig.SensorToMechanismRatio = Constants.INTAKE_ROTS_TO_INCHES;

		var slot0Configs = talonFXConfigs.Slot0;
		slot0Configs.GravityType = GravityTypeValue.Elevator_Static;
		slot0Configs.kG = Constants.PIVOT_KG;
		slot0Configs.kS = Constants.PIVOT_KS;
		slot0Configs.kV = Constants.PIVOT_KV;
		slot0Configs.kA = Constants.PIVOT_KA;
		slot0Configs.kP = Constants.PIVOT_KP;
		slot0Configs.kI = Constants.PIVOT_KI;
		slot0Configs.kD = Constants.PIVOT_KD;
		slot0Configs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

		var slot1Configs = intakeConfigs.Slot0;
		slot1Configs.GravityType = GravityTypeValue.Elevator_Static;
		slot1Configs.kG = Constants.INTAKE_KG;
		slot1Configs.kS = Constants.INTAKE_KS;
		slot1Configs.kV = Constants.INTAKE_KV;
		slot1Configs.kA = Constants.INTAKE_KA;
		slot1Configs.kP = Constants.INTAKE_KP;
		slot1Configs.kI = Constants.INTAKE_KI;
		slot1Configs.kD = Constants.INTAKE_KD;
		slot1Configs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

		var pivotMotionMagicConfigs = talonFXConfigs.MotionMagic;
		pivotMotionMagicConfigs.MotionMagicCruiseVelocity = Constants.INTAKE_CRUISE_VELO;
		pivotMotionMagicConfigs.MotionMagicAcceleration = Constants.INTAKE_TARGET_ACCEL;
		pivotMotionMagicConfigs.MotionMagicExpo_kV = Constants.INTAKE_EXPO_KV;

		var intakeMotionMagicConfigs = intakeConfigs.MotionMagic;
		intakeMotionMagicConfigs.MotionMagicCruiseVelocity = Constants.INTAKE_CRUISE_VELO;
		intakeMotionMagicConfigs.MotionMagicAcceleration = Constants.INTAKE_TARGET_ACCEL;
		intakeMotionMagicConfigs.MotionMagicExpo_kV = Constants.INTAKE_EXPO_KV;


		pivotMotorLeft.getConfigurator().apply(talonFXConfigs);

		BaseStatusSignal.setUpdateFrequencyForAll(
				Constants.UPDATE_FREQUENCY_HZ,
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
				Constants.UPDATE_FREQUENCY_HZ,
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
				Constants.UPDATE_FREQUENCY_HZ,
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

	@Override
	public void reset() {
		setCurrentState(FSMState.IDLE_IN_STATE);

		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}

	@Override
	public void update(TeleopInput input) {
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
		setCurrentState(nextState(input));
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


	@Override
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

	@Override
	protected FSMState nextState(TeleopInput input) {
		switch (getCurrentState()) {
			case IDLE_IN_STATE:
				if (input.isFoldOutButtonPressed()) {
					return FSMState.FOLD_OUT_STATE;
				} else if (input.isPartialOutButtonPressed()) {
					return FSMState.PARTIAL_OUT_STATE;
				} else {
					return FSMState.IDLE_IN_STATE;
				}

			case FOLD_OUT_STATE:
				if (isBottomLimitReached()) {
					return FSMState.IDLE_OUT_STATE;
				} else {
					return FSMState.FOLD_OUT_STATE;
				}

			case PARTIAL_OUT_STATE:
				if (input.isFoldOutButtonPressed()) {
					return FSMState.FOLD_OUT_STATE;
				} else if (input.isFoldInButtonPressed()) {
					return FSMState.FOLD_IN_STATE;
				} else {
					return FSMState.PARTIAL_OUT_STATE;
				}

			case IDLE_OUT_STATE:
				if (input.isIntakeButtonPressed()) {
					return FSMState.INTAKE_STATE;
				} else if (input.isOuttakeButtonPressed()) {
					return FSMState.OUTTAKE_STATE;
				} else if (input.isFoldInButtonPressed()) {
					return FSMState.FOLD_IN_STATE;
				} else if (input.isPartialOutButtonPressed()) {
					return FSMState.PARTIAL_OUT_STATE;
				} else {
					return FSMState.IDLE_OUT_STATE;
				}

			case INTAKE_STATE:
				if (input.isIntakeButtonReleased()) {
					return FSMState.IDLE_OUT_STATE;
				} else {
					return FSMState.INTAKE_STATE;
				}

			case OUTTAKE_STATE:
				if (input.isOuttakeButtonReleased()) {
					return FSMState.IDLE_OUT_STATE;
				} else {
					return FSMState.OUTTAKE_STATE;
				}

			case FOLD_IN_STATE:
				if (isTopLimitReached()) {
					return FSMState.IDLE_IN_STATE;
				} else {
					return FSMState.FOLD_IN_STATE;
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
		pivotMotorRight.setControl(pivotMotionRequest.withPosition(Constants.INTAKE_GROUND_TARGET));
	}
	/**
	 * Handle behavior in PARTIAL_OUT_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handlePartialOutState(TeleopInput input) {
		pivotMotorRight.setControl(pivotMotionRequest.withPosition(Constants.PARTIAL_OUT_POSITION));
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
		intakeMotor.setControl(intakeMotionRequest.withVelocity(Constants.INTAKE_TARGET_VELOCITY));
	}
	/**
	 * Handle behavior in START_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleOuttakeState(TeleopInput input) {
		intakeMotor.setControl(intakeMotionRequest.withVelocity(Constants.OUTTAKE_TARGET_VELOCITY));
	}
	/**
	 * Handle behavior in OTHER_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleFoldInState(TeleopInput input) {
		pivotMotorRight.setControl(pivotMotionRequest.withPosition(Constants.INTAKE_UPPER_TARGET));
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
