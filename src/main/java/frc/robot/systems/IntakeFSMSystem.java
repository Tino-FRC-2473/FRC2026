package frc.robot.systems;

import org.littletonrobotics.junction.AutoLogOutput;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import frc.robot.input.Input;
// Robot Imports
import frc.robot.input.InputTypes.ButtonInput;
import frc.robot.motors.TalonFXWrapper;
import frc.robot.Constants.IntakeConstants;
import frc.robot.HardwareMap;


public class IntakeFSMSystem extends FSMSystem<IntakeFSMSystem.IntakeFSMState> {
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

	private MotionMagicVoltage pivotMotionRequest;
	private MotionMagicVelocityVoltage intakeMotionRequest;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private TalonFXWrapper pivotMotorLeft;
	private TalonFXWrapper pivotMotorRight;
	private TalonFXWrapper intakeMotor;
	private DigitalInput groundLimitSwitch;
	private DigitalInput topLimitSwitch;

	private Angle posRadians;


	private TalonFXSimState pivotSimState;
	private SingleJointedArmSim intakeSim;
	private DIOSim simGroundLimitSwitch;
	private DIOSim simTopLimitSwitch;



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

		pivotSimState = pivotMotorRight.getSimState();


		intakeMotor = new TalonFXWrapper(HardwareMap.CAN_ID_SPARK_INTAKE);

		var talonFXConfigs = new TalonFXConfiguration();
		var intakeConfigs = new TalonFXConfiguration();

		pivotMotorLeft.setControl(new Follower(pivotMotorRight.getDeviceID(),
			MotorAlignmentValue.Aligned));

		var pivotConfig = talonFXConfigs.Feedback;
		pivotConfig.SensorToMechanismRatio = IntakeConstants.INTAKE_PIVOT_GEARING;

		var limitConfig = talonFXConfigs.CurrentLimits;
		limitConfig.StatorCurrentLimit = IntakeConstants.PIVOT_CURRENT_LIMIT;
		limitConfig.StatorCurrentLimitEnable = true;

		var outputConfigs = talonFXConfigs.MotorOutput;
		outputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

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

		var intakeConfig = intakeConfigs.Feedback;
		intakeConfig.SensorToMechanismRatio = IntakeConstants.INTAKE_GEARING;

		var slot1Configs = intakeConfigs.Slot0;
		slot1Configs.kV = IntakeConstants.INTAKE_KV;
		slot1Configs.kA = IntakeConstants.INTAKE_KA;
		slot1Configs.kP = IntakeConstants.INTAKE_KP;
		slot1Configs.kI = IntakeConstants.INTAKE_KI;
		slot1Configs.kD = IntakeConstants.INTAKE_KD;
		slot1Configs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

		var pivotMotionMagicConfigs = talonFXConfigs.MotionMagic;
		pivotMotionMagicConfigs.MotionMagicCruiseVelocity = IntakeConstants.PIVOT_CRUISE_VELO;
		pivotMotionMagicConfigs.MotionMagicAcceleration = IntakeConstants.PIVOT_TARGET_ACCEL;
		pivotMotionMagicConfigs.MotionMagicExpo_kV = IntakeConstants.PIVOT_EXPO_KV;

		var intakeMotionMagicConfigs = intakeConfigs.MotionMagic;
		intakeMotionMagicConfigs.MotionMagicCruiseVelocity = IntakeConstants.INTAKE_CRUISE_VELO;
		intakeMotionMagicConfigs.MotionMagicAcceleration = IntakeConstants.INTAKE_TARGET_ACCEL;
		intakeMotionMagicConfigs.MotionMagicExpo_kV = IntakeConstants.INTAKE_EXPO_KV;

		pivotMotorRight.getConfigurator().apply(talonFXConfigs);

		BaseStatusSignal.setUpdateFrequencyForAll(
				IntakeConstants.UPDATE_FREQUENCY,
				pivotMotorRight.getPosition(),
				pivotMotorRight.getVelocity(),
				pivotMotorRight.getAcceleration(),
				pivotMotorRight.getMotorVoltage(),
				pivotMotorRight.getPosition(),
				pivotMotorRight.getVelocity()
		);

		pivotMotorRight.optimizeBusUtilization();

		pivotMotorLeft.getConfigurator().apply(talonFXConfigs);

		BaseStatusSignal.setUpdateFrequencyForAll(
				IntakeConstants.UPDATE_FREQUENCY,
				pivotMotorLeft.getPosition(),
				pivotMotorLeft.getVelocity(),
				pivotMotorLeft.getAcceleration(),
				pivotMotorLeft.getMotorVoltage(),
				pivotMotorLeft.getPosition(),
				pivotMotorLeft.getVelocity()
		);

		pivotMotorLeft.optimizeBusUtilization();

		intakeMotor.getConfigurator().apply(intakeConfigs);

		BaseStatusSignal.setUpdateFrequencyForAll(
				IntakeConstants.UPDATE_FREQUENCY,
				intakeMotor.getPosition(),
				intakeMotor.getVelocity(),
				intakeMotor.getAcceleration(),
				intakeMotor.getMotorVoltage()
		);

		intakeMotor.optimizeBusUtilization();


		//initialize limit switch
		groundLimitSwitch = new DigitalInput(HardwareMap.INTAKE_GROUND_LIMIT_SWITCH_DIO_PORT);
		topLimitSwitch = new DigitalInput(HardwareMap.INTAKE_TOP_LIMIT_SWITCH_DIO_PORT);

		// Set motor positions
		pivotMotorLeft.setPosition(IntakeConstants.UPPER_TARGET_ANGLE);
		pivotMotorRight.setPosition(IntakeConstants.UPPER_TARGET_ANGLE);
		intakeMotor.setPosition(0);

		/*
		if (RobotBase.isSimulation()) {
			intakeSim = new SingleJointedArmSim(DCMotor.getKrakenX60(2),
				IntakeConstants.INTAKE_PIVOT_GEARING,
				IntakeConstants.J,
				IntakeConstants.PIVOT_ARM_LENGTH,
				IntakeConstants.PIVOT_MIN_ROTATION,
				IntakeConstants.PIVOT_MAX_ROTATION,
				true,
				0);
			simGroundLimitSwitch = new DIOSim(groundLimitSwitch);
			simTopLimitSwitch = new DIOSim(topLimitSwitch);
		}*/

		// Reset state machine
		reset();
	}

	/* ======================== Public methods ======================== */

	// overridden methods don't require javadocs
	// however, you may want to add implementation specific javadocs


	/**
	 * resets the FSM_STATE.
	 */
	public void reset() {
		setCurrentState(IntakeFSMState.IDLE_IN_STATE);

		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}

	/**
	 * updates the current state in IntakeFSMState.
	 * @param input
	 */
	public void update(Input input) {
		if (RobotBase.isSimulation()) {
			//posRadians = Units.Radians.of(intakeSim.getAngleRads());


			// In this method, we update our simulation of what our elevator is doing
			// First, we set our "inputs" (voltages)
			intakeSim.setInput(pivotMotorRight.get() * RobotController.getBatteryVoltage());
			// Next, we update it. The standard loop time is 20ms.
			intakeSim.update(IntakeConstants.SIM_UPDATE_SECONDS);
			// Finally, we set our simulated encoder's readings and simulated battery voltage
			pivotMotorRight.setPosition(intakeSim.getAngleRads());

			/*
			//update limit switch sims
			boolean atTop = posRadians <= IntakeConstants.SIM_LIMIT_SWITCH_BUFFER;
			boolean atBottom = posRadians >= IntakeConstants.SIM_LIMIT_SWITCH_BUFFER;
			simGroundLimitSwitch.setValue(position);
			simTopLimitSwitch.setValue(atTop);
			*/

			// SimBattery estimates loaded battery voltages
			RoboRioSim.setVInVoltage(
				BatterySim.calculateDefaultBatteryLoadedVoltage(intakeSim.getCurrentDrawAmps()));
		}
		if (input == null) {
			return;
		}
		switch (getCurrentState()) {
			case IDLE_IN_STATE:
				handleIdleState(input);
				break;

			case FOLD_OUT_STATE:
				handleFoldOutState(input);
				break;

			case IDLE_OUT_STATE:
				handleIdleState(input);
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
	 * Getter for intake current state.
	 * @return intake current state.
	*/
	@AutoLogOutput(key = "Intake/Intake Current State")
	public IntakeFSMState getIntakeState() {
		return getCurrentState();
	}


	/**
	 * Getter for intake motor velocity.
	 * @return intake motor velocity as a double
	 */
	@AutoLogOutput(key = "Intake/Intake Motor Vel", unit = "rps")
	public double getIntakeMotorVelocity() {
		return intakeMotor.getVelocity().getValueAsDouble();
	}

	/**
	 * Getter for pivot motor velocity.
	 * @return pivot motor velocity as a double
	 */
	@AutoLogOutput(key = "Intake/Pivot Motor Vel", unit = "rps")
	public double getPivotMotorVelocity() {
		return pivotMotorRight.getVelocity().getValueAsDouble();
	}

	/**
	 * Getter for pivot motor position.
	 * @return intake motor position as a double
	 */
	@AutoLogOutput(key = "Intake/Pivot Motor Pos", unit = "radians")
	public double getPivotMotorPos() {
		return pivotMotorRight.getPosition().getValueAsDouble();
	}

	/**
	 * Getter for intake motor voltage.
	 * @return intake motor voltage as a double
	 */
	@AutoLogOutput(key = "Intake/Intake Motor Voltage", unit = "volts")
	public double getIntakeMotorVolatge() {
		return intakeMotor.getMotorVoltage().getValueAsDouble();
	}

	/**
	 * Getter for pivot motor left voltage.
	 * @return pivot motor left voltage as a double
	 */
	@AutoLogOutput(key = "Intake/Pivot Motor Left Voltage", unit = "volts")
	public double getPivotMotorLeftVolatge() {
		return pivotMotorLeft.getMotorVoltage().getValueAsDouble();
	}

	/**
	 * Getter for pivot motor right voltage.
	 * @return pivot motor right voltage as a double
	 */
	@AutoLogOutput(key = "Intake/Pivot Motor Right Voltage", unit = "volts")
	public double getPivotMotorRightVoltage() {
		return pivotMotorRight.getMotorVoltage().getValueAsDouble();
	}

	/**
	 * Getter for pivot motor right current.
	 * @return pivot motor right current as a double
	 */
	@AutoLogOutput(key = "Intake/Pivot Motor Right Current", unit = "amps")
	public double getPivotMotorRightCurrent() {
		return pivotMotorRight.getStatorCurrent().getValueAsDouble();
	}

	/**
	 * Getter for intake motor voltage.
	 * @return intake motor voltage as a double
	 */
	@AutoLogOutput(key = "Intake/Is at Bottom Limit?")
	public boolean isAtBottomLimit() {
		return isBottomLimitReached();
	}

	/**
	 * Getter for intake motor voltage.
	 * @return intake motor voltage as a double
	 */
	@AutoLogOutput(key = "Intake/Is at Top Limit?")
	public boolean isAtTopLimit() {
		return isTopLimitReached();
	}


	/* ======================== Protected methods ======================== */


	protected IntakeFSMState nextState(Input input) {
		if (input == null) {
			return IntakeFSMState.IDLE_IN_STATE;
		}
		switch (getCurrentState()) {
			case IDLE_IN_STATE:
				if (input.getButtonPressed(ButtonInput.FOLD_OUT_BUTTON)) {
					return IntakeFSMState.FOLD_OUT_STATE;
				} else if (input.getButtonPressed(ButtonInput.PARTIAL_OUT_BUTTON)) {
					return IntakeFSMState.PARTIAL_OUT_STATE;
				} else {
					return IntakeFSMState.IDLE_IN_STATE;
				}

			case FOLD_OUT_STATE:
				if (isBottomLimitReached()) {
					return IntakeFSMState.IDLE_OUT_STATE;
				} else if (input.getButtonPressed(ButtonInput.FOLD_IN_BUTTON)) {
					return IntakeFSMState.FOLD_IN_STATE;
				} else if (input.getButtonPressed(ButtonInput.PARTIAL_OUT_BUTTON)) {
					return IntakeFSMState.PARTIAL_OUT_STATE;
				} else {
					return IntakeFSMState.FOLD_OUT_STATE;
				}

			case PARTIAL_OUT_STATE:
				if (input.getButtonPressed(ButtonInput.FOLD_OUT_BUTTON)) {
					return IntakeFSMState.FOLD_OUT_STATE;
				} else if (input.getButtonPressed(ButtonInput.FOLD_IN_BUTTON)) {
					return IntakeFSMState.FOLD_IN_STATE;
				} else {
					return IntakeFSMState.PARTIAL_OUT_STATE;
				}

			case IDLE_OUT_STATE:
				if (input.getButtonPressed(ButtonInput.INTAKE_BUTTON)
					&& !input.getButtonPressed(ButtonInput.OUTTAKE_BUTTON)
					&& !input.getButtonPressed(ButtonInput.FOLD_IN_BUTTON)
					&& !input.getButtonPressed(ButtonInput.FOLD_OUT_BUTTON)) {
					return IntakeFSMState.INTAKE_STATE;
				} else if (input.getButtonPressed(ButtonInput.OUTTAKE_BUTTON)
					&& !input.getButtonPressed(ButtonInput.INTAKE_BUTTON)
					&& !input.getButtonPressed(ButtonInput.FOLD_IN_BUTTON)
					&& !input.getButtonPressed(ButtonInput.FOLD_OUT_BUTTON)) {
					return IntakeFSMState.OUTTAKE_STATE;
				} else if (input.getButtonPressed(ButtonInput.FOLD_IN_BUTTON)) {
					return IntakeFSMState.FOLD_IN_STATE;
				} else if (input.getButtonPressed(ButtonInput.PARTIAL_OUT_BUTTON)) {
					return IntakeFSMState.PARTIAL_OUT_STATE;
				} else {
					return IntakeFSMState.IDLE_OUT_STATE;
				}

			case INTAKE_STATE:
				if (input.getButtonReleased(ButtonInput.INTAKE_BUTTON)) {
					return IntakeFSMState.IDLE_OUT_STATE;
				} else {
					return IntakeFSMState.INTAKE_STATE;
				}

			case OUTTAKE_STATE:
				if (input.getButtonReleased(ButtonInput.OUTTAKE_BUTTON)) {
					return IntakeFSMState.IDLE_OUT_STATE;
				} else {
					return IntakeFSMState.OUTTAKE_STATE;
				}

			case FOLD_IN_STATE:
				if (isTopLimitReached()) {
					return IntakeFSMState.IDLE_IN_STATE;
				} else if (input.getButtonPressed(ButtonInput.FOLD_OUT_BUTTON)) {
					return IntakeFSMState.FOLD_OUT_STATE;
				} else if (input.getButtonPressed(ButtonInput.PARTIAL_OUT_BUTTON)) {
					return IntakeFSMState.PARTIAL_OUT_STATE;
				}  else {
					return IntakeFSMState.FOLD_IN_STATE;
				}

			default:
				throw new IllegalStateException("Invalid state: " + getCurrentState().toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in idle states.
	 * @param input Global Input if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleIdleState(Input input) {
		pivotMotorRight.set(0);
		intakeMotor.setControl(intakeMotionRequest.
			withVelocity(0));
	}
	/**
	 * Handle behavior in FOLD_OUT_STATE.
	 * @param input Global Input if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleFoldOutState(Input input) {
		pivotMotorRight.setControl(pivotMotionRequest.
			withPosition(IntakeConstants.GROUND_TARGET_ANGLE));
	}
	/**
	 * Handle behavior in PARTIAL_OUT_STATE.
	 * @param input Global Input if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handlePartialOutState(Input input) {
		if (Math.abs(IntakeConstants.PARTIAL_OUT_TARGET_ANGLE.in(Rotations)
			- pivotMotorRight.getPosition().getValueAsDouble()) >= IntakeConstants.PIVOT_BUFFER) {
			pivotMotorRight.setControl(pivotMotionRequest.
				withPosition(IntakeConstants.PARTIAL_OUT_TARGET_ANGLE));
		} else {
			pivotMotorRight.set(0);
		}
	}
	/**
	 * Handle behavior in INTAKE_STATE.
	 * @param input Global Input if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleIntakeState(Input input) {
		intakeMotor.setControl(intakeMotionRequest.
			withVelocity(IntakeConstants.INTAKE_TARGET_VELOCITY));
	}
	/**
	 * Handle behavior in OUTTAKE_STATE.
	 * @param input Global Input if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleOuttakeState(Input input) {
		intakeMotor.setControl(intakeMotionRequest.
			withVelocity(IntakeConstants.OUTTAKE_TARGET_VELOCITY));
	}
	/**
	 * Handle behavior in FOLD_IN_STATE.
	 * @param input Global Input if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleFoldInState(Input input) {
		pivotMotorRight.setControl(pivotMotionRequest.
			withPosition(IntakeConstants.UPPER_TARGET_ANGLE));
	}



	/* ======================== Private methods ======================== */
	/**
	 * Getter for the result of the elevator's bottom limit switch.
	 * @return whether the limit is reached
	 */
	private boolean isBottomLimitReached() {
		if (RobotBase.isSimulation()) {
			return pivotMotorRight.getPosition().getValueAsDouble()
				<= IntakeConstants.GROUND_TARGET_ANGLE.in(Radians);
		}
		return groundLimitSwitch.get(); // switch is normally open
	}

	/**
	 * Getter for the result of the elevator's top limit switch.
	 * @return whether the limit is reached
	 */
	private boolean isTopLimitReached() {
		if (RobotBase.isSimulation()) {
			return pivotMotorRight.getPosition().getValueAsDouble()
				>= IntakeConstants.UPPER_TARGET_ANGLE.in(Radians);
		}
		return topLimitSwitch.get(); // switch is normally open
	}

	/**
	 * Getter for the result of the elevator's top limit switch.
	 * @return whether the limit is reached
	 */
	public boolean isIntakeDownRunning() {
		return (getCurrentState() == IntakeFSMState.INTAKE_STATE); 
	}

}
