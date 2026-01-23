package frc.robot.systems;

// WPILib Imports

// Third party Hardware Imports
import com.revrobotics.spark.SparkMax;

import frc.robot.motors.SparkMaxWrapper;
import frc.robot.HardwareMap;
import frc.robot.input.Input;

// enum FSMState {
// 	START_STATE,
// 	OTHER_STATE
// }

public class ExampleFSMSystem extends FSMSystem<ExampleFSMSystem.FSMState> {

	public enum FSMState {
		START_STATE,
		OTHER_STATE
	}


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
	public ExampleFSMSystem() {
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
		setCurrentState(FSMState.START_STATE);

		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}

	@Override
	public void update(Input input) {
		switch (getCurrentState()) {
			case START_STATE:
				handleStartState(input);
				break;

			case OTHER_STATE:
				handleOtherState(input);
				break;

			default:
				throw new IllegalStateException("Invalid state: " + getCurrentState().toString());
		}
		setCurrentState(nextState(input));
	}

	/* ======================== Protected methods ======================== */

	@Override
	protected FSMState nextState(Input input) {
		switch (getCurrentState()) {
			case START_STATE:
				if (input != null) {
					return FSMState.OTHER_STATE;
				} else {
					return FSMState.START_STATE;
				}

			case OTHER_STATE:
				return FSMState.OTHER_STATE;

			default:
				throw new IllegalStateException("Invalid state: " + getCurrentState().toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in START_STATE.
	 * @param input Global Input if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleStartState(Input input) {
		exampleMotor.set(0);
	}
	/**
	 * Handle behavior in OTHER_STATE.
	 * @param input Global Input if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleOtherState(Input input) {
		exampleMotor.set(MOTOR_RUN_POWER);
	}

}
