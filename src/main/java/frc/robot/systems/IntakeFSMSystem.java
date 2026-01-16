package frc.robot.systems;

// WPILib Imports

// Third party Hardware Imports
import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.google.flatbuffers.Constants;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.motors.SparkMaxWrapper;
import frc.robot.motors.TalonFXWrapper;
import frc.robot.HardwareMap;
import frc.robot.constants.Constants;
import frc.robot.Robot;
import frc.robot.systems.AutoHandlerSystem.AutoFSMState;

enum FSMState {
	IDLE_IN_STATE,
	FOLD_OUT_STATE,
	IDLE_OUT_STATE,
	INTAKE_STATE,
	OUTTAKE_STATE,
	FOLD_IN_STATE
}

public class IntakeFSMSystem extends FSMSystem<FSMState> {
	/* ======================== Constants ======================== */

	private static final float MOTOR_RUN_POWER = 0.1f;

	/* ======================== Private variables ======================== */

	private MotionMagicVoltage motionRequest;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private SparkMax exampleMotor;
	private TalonFXWrapper pivotMotorLeft;
	private TalonFXWrapper pivotMotorRight;
	private TalonFXWrapper intakeMotor;
	private DigitalInput groundLimitSwitch;
	

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public IntakeFSMSystem() {

		motionRequest = new MotionMagicVoltage(0);

		// Perform hardware init using a wrapper class
		// this is so we can see motor outputs during simulatiuons
		exampleMotor = new SparkMaxWrapper(HardwareMap.CAN_ID_SPARK_SHOOTER,
										SparkMax.MotorType.kBrushless);

		//initialize motors
		pivotMotorLeft = new TalonFXWrapper(HardwareMap.CAN_ID_SPARK_PIVOT_LEFT);
		pivotMotorRight = new TalonFXWrapper(HardwareMap.CAN_ID_SPARK_PIVOT_RIGHT);
		intakeMotor = new TalonFXWrapper(HardwareMap.CAN_ID_SPARK_INTAKE);

		//initialize limit switch
		groundLimitSwitch = new DigitalInput(HardwareMap.INTAKE_GROUND_LIMIT_SWITCH_DIO_PORT);

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

			default:
				throw new IllegalStateException("Invalid state: " + getCurrentState().toString());
		}
		setCurrentState(nextState(input));
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
				if (input.isFoldButtonPressed()) {
					return FSMState.FOLD_OUT_STATE;
				} else {
					return FSMState.IDLE_IN_STATE;
				}

			case FOLD_OUT_STATE:
				if (isBottomLimitReached()) {
					return FSMState.IDLE_OUT_STATE;
				} else {
					return FSMState.FOLD_OUT_STATE;
				}

			case IDLE_OUT_STATE:
				if (input.isIntakeButtonPressed()) {
					return FSMState.INTAKE_STATE;
				} else if(input.isOuttakeButtonPressed()){
					return FSMState.OUTTAKE_STATE;
				} else if(input.isFoldButtonPressed()){
					return FSMState.FOLD_IN_STATE;
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
				if (input != null) {
					return FSMState.OTHER_STATE;
				} else {
					return FSMState.START_STATE;
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
		pivotMotorLeft.setControl(motionRequest.withPosition(Constants.));
		pivotMotorLeft.setControl(motionRequest.withPosition(Constants.));
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
	}
	/**
	 * Handle behavior in START_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleOuttakeState(TeleopInput input) {
	}
	/**
	 * Handle behavior in OTHER_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleFoldInState(TeleopInput input) {
		pivotMotorLeft.setControl(motionRequest.withPosition(Constants.pos));
		pivotMotorLeft.setControl(motionRequest.withPosition(Constants.));
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

}
