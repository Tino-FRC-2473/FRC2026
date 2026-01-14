package frc.robot.systems;

// WPILib Imports

// Third party Hardware Imports
import com.revrobotics.spark.SparkMax;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.motors.SparkMaxWrapper;
import frc.robot.HardwareMap;
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

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private SparkMax exampleMotor;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public IntakeFSMSystem() {
		// Perform hardware init using a wrapper class
		// this is so we can see motor outputs during simulatiuons
		exampleMotor = new SparkMaxWrapper(HardwareMap.CAN_ID_SPARK_SHOOTER,
										SparkMax.MotorType.kBrushless);

		// Reset state machine
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
				if (input != null) {
					return FSMState.OTHER_STATE;
				} else {
					return FSMState.START_STATE;
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
}
