package frc.robot.systems;

// Libary imports
import edu.wpi.first.wpilibj2.command.Command;

// Robot imports
import frc.robot.input.Input;

/**
 * A superclass that all FSM systems should inherit from.
 * @param <S> the enum containing all FSM states
 */
public abstract class FSMSystem<S> {

	/* ======================== Public methods ======================== */

	// Current FSM state
	private S currentState;

	/* ======================== Public methods ======================== */

	/**
	 * Get the current FSM state.
	 * @return the current FSM state
	 */
	public S getCurrentState() {
		return currentState;
	}

	/**
	 * Set the current FSM state.
	 * @param newState the new state
	 */
	public void setCurrentState(S newState) {
		currentState = newState;
	}

	/**
	 * returns an ObservedStateCommand for the given states.
	 * @param states the set of states to scan for
	 * @return the command
	 */
	public ObservedStateCommand watchForStatesCommand(
		@SuppressWarnings("unchecked") S... states) {
		return new ObservedStateCommand(states);
	}

	/**
	 * Reset this system to its start state. This may be called from mode init
	 * when the robot is enabled.
	 *
	 * Note this is distinct from the one-time initialization in the constructor
	 * as it may be called multiple times in a boot cycle,
	 * Ex. if the robot is enabled, disabled, then reenabled.
	 */
	public abstract void reset();

	/**
	 * Update FSM based on new inputs. This function only calls the FSM state
	 * specific handlers.
	 * @param input Global Input reflecting the input to the robot
	 */
	public abstract void update(Input input);

	/**
	 * Decide the next state to transition to. This is a function of the inputs
	 * and the current state of this FSM. This method should not have any side
	 * effects on outputs. In other words, this method should only read or get
	 * values to decide what state to go to.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *		the robot is in autonomous mode.
	 * @return FSM state for the next iteration
	 */
	protected abstract S nextState(Input input);

	private final class ObservedStateCommand extends Command {

		private S[] endStateSequence;
		private int numMatchingStates;

		/**
		 * creates an ObservedStateCommand, which is a command that does nothing
		 * and ends when a particular sequence of states is observed.
		 * @param states the sequence of states that triggers the command to end
		 */
		private ObservedStateCommand(@SuppressWarnings("unchecked") S... states) {
			endStateSequence = states;
			numMatchingStates = 0;
		}

		@Override
		public void execute() {
			if (getCurrentState() == endStateSequence[numMatchingStates]) {
				numMatchingStates++;
			} else if (numMatchingStates > 0
					&& getCurrentState() == endStateSequence[numMatchingStates - 1]) {
				numMatchingStates = 0;
			}
		}

		@Override
		public boolean isFinished() {
			return numMatchingStates >= endStateSequence.length;
		}
	}

}
