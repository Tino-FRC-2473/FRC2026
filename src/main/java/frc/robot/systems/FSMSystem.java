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

	/* ======================== Private variables ======================== */

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
	 * Get a command that waits for a sequence of states to be observed.
	 * @param states the sequence of states
	 * @return the command
	 */
	public ObservedStateCommand watchForStatesCommand(
			@SuppressWarnings("unchecked") S... states) {
		return new ObservedStateCommand(states);
	}

	/* ======================== Abstract methods ======================== */

	/**
	 * Reset the current FSM state.
	 */
	public abstract void reset();

	/**
	 * Update the FSM state periodically based on inputs.
	 * @param input the input object
	 */
	public abstract void update(Input input);

	/**
	 * Get the next state for the FSM based on inputs.
	 * @param input the input object
	 * @return the next state
	 */
	protected abstract S nextState(Input input);

	/* ======================== Command classes ======================== */

	private final class ObservedStateCommand extends Command {
		private S[] targetSequence;
		private int sequenceProgress;

		/**
		 * Create a command that waits for a sequence of states to be observed.
		 * @param states the sequence of states
		 */
		private ObservedStateCommand(@SuppressWarnings("unchecked") S... states) {
			targetSequence = states;
			sequenceProgress = 0;
		}

		@Override
		public void execute() {
			if (getCurrentState() == targetSequence[sequenceProgress]) {
				sequenceProgress++;
			} else if (sequenceProgress > 0
					&& getCurrentState() == targetSequence[sequenceProgress - 1]) {
				sequenceProgress = 0;
			}
		}

		@Override
		public boolean isFinished() {
			return sequenceProgress >= targetSequence.length;
		}
	}

}
