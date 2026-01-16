package frc.robot.systems;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.TeleopInput;
import frc.robot.systems.FSMSystem.ObservedStateCommand.ExitBehavior;

/**
 * This is a superclass for FSMs with NECCESARY methods to implement
 *
 * Start implementing an FSM by writing this in a new java file:
 *
 * <code>
 * enum FSMState {
 *	  // add states here
 * }
 * public class _______ extends FSMSystem&lt;FSMState&gt; {
 *	  ...
 * }
 * </code>
 *
 * Your compiler / IDE will tell you what methods you need to implement
 * You should also have state handlers shown in the example
 * @param <S> the type of state
 */
public abstract class FSMSystem<S> {

	/**
	 * A variable used to ensure that multiple commands for the same FSM cannot be paralelized.
	 */
	private boolean isObserved = false;

	/**
	 * A command that wraps an 'observed' stateful process.
	 * The command will request an entry state, and will execute until
	 * a state transition out of a specific exit state.
	 * 
	 * 
	 * Example: If the mechanism is a shooter, the entry state is RAMP_UP,
	 * and the exit state is IDLE. Any number of additional states may be invoked
	 * in the process of getting from RAMP_UP to IDLE. In this case we would want the
	 * END_ON_ENTRY mode since we would likely start the command in IDLE,
	 * and want it to end when  we return to IDLE.
	 * 
	 */
	public class ObservedStateCommand extends Command {

		/**
		 * An enum specifying whether the ObservedStateCommand
		 * should be finished when the exit state is entered or exited.
		 */
		public enum ExitBehavior {
			/**
			 * end on entry mode, which ends the command when the
			 * current state changes from any other state to the exit state.
			 * Ex: Command ends when we enter IDLE
			 */
			END_ON_ENTRY,

			/**
			 * end on entry mode, which ends the command when the
			 * current state changes from the exit state to any other state.
			 * Ex: Command ends when we leave ALIGN
			 */
			END_ON_EXIT
		}

		/** The FSM state on the previous tick, used to detect state transitions */
		private S previousState;

		/** Whether the command is finished. */
		private boolean isFinished;

		/** Whether the observation process should end on */
		private final ExitBehavior exitMode;

		/**
		 * The wanted state, which denotes the FSM's goal.
		 * This will switch to the observedState after the original goal state has been reached.
		 */
		private S wantedState;

		/**
		 * The state that will trigger the command to finish
		 */
		private final S observedState;

		/**
		 * Constructs a ObservedStateCommand given the goal state, which is usually
		 * the intended behavior the command is supposed to achieve, and information about
		 * how to end the command.
		 * @param goalState the goal state / the action the command should achieve
		 * @param exitState a state that dictates how the command will end
		 * @param exitType an enum value determining which type of state transition related to exitState will end the command.
		 */
		private ObservedStateCommand(S goalState, S exitState, ExitBehavior exitType) {
			this.wantedState = goalState;
			this.observedState = exitState;
			this.exitMode = exitType;
		}

		/**
		 * Constructs a ObservedStateCommand for quicker one off actions, where the
		 * command is intended to when the target state is reached or exited.
		 * (the goal / target state is the same as the exit state)
		 * @param goalState The goal state and the exit state
		 * @param exitType an enum value determining which type of state transition related to exitState will end the command.
		 */
		private ObservedStateCommand(S goalState, ExitBehavior exitType) {
			this(goalState, goalState, exitType);
		}

		@Override
		public void initialize() {
			super.initialize();
			if (isObserved) {
				throw new IllegalStateException("Same FSM observed state commands cannot be parralelized");
			}
			isObserved = true;
			setCurrentState(wantedState);
		}

		@Override
		public void execute() {
			super.execute();
			isFinished = previousState != null && getCurrentState() != previousState && switch (exitMode) {
				case END_ON_ENTRY -> getCurrentState() == observedState;
				case END_ON_EXIT -> previousState == observedState;
			} || super.isFinished();
			previousState = getCurrentState();
			if (getCurrentState() == wantedState) {
				wantedState = observedState;
			}
		}

		@Override
		public boolean isFinished() {
			return isFinished;
		}

		@Override
		public void end(boolean interrupted) {
			super.end(interrupted);
			isObserved = false;
		}

	}

	/**
	 * Constructs and returns an observed state command with END_ON_ENTRY exit behavior.
	 * @param goalState the goal state.
	 * @param exitState the exit state.
	 * @return the command.
	 */
	public Command getStateProcessCommand(S goalState, S exitState) {
		return new ObservedStateCommand(goalState, exitState, ExitBehavior.END_ON_ENTRY);
	}

	/**
	 * Constructs and returns an observed state command with END_ON_ENTRY exit behavior.
	 * @param goalState the goal & exit state.
	 * @return the command.
	 */
	public Command getStateCommand(S state) {
		return new ObservedStateCommand(state, state, ExitBehavior.END_ON_ENTRY);
	}

	/**
	 * Constructs and returns an observed state command with specified exit behavior.
	 * @param goalState the goal & exit state.
	 * @param exitBehavior the exit behavior.
	 * @return the command.
	 */
	public Command getStateProcessCommand(S goalState, S exitState, ExitBehavior exitBehavior) {
		return new ObservedStateCommand(goalState, exitState, exitBehavior);
	}

	/**
	 * Constructs and returns an observed state command with specified exit behavior.
	 * @param goalState the goal state.
	 * @param exitState the exit state.
	 * @param exitBehavior the exit behavior.
	 * @return the command.
	 */
	public Command getStateCommand(S state, ExitBehavior exitBehavior) {
		return new ObservedStateCommand(state, state, exitBehavior);
	}

	/**
	 * the current state, defined as part of the provided statespace.
	 */
	private S currentState;

	/**
	 * The wanted state, which is an externally set value that the FSM can use to make state transition decisions.
	 */
	private S wantedState;

	/**
	 * Return current FSM state.
	 * @return Current FSM state
	 */
	public S getCurrentState() {
		return currentState;
	}

	/**
	 * Sets the current state.
	 * @param newState the new state
	 */
	protected void setCurrentState(S newState) {
		currentState = newState;
	}

	/**
	 * Return wanted FSM state.
	 * @return Wanted FSM state
	 */
	public S getWantedState() {
		return wantedState;
	}

	/**
	 * Sets the wanted state.
	 * @param newState the new state
	 */
	private void setWantedState(S newState) {
		wantedState = newState;
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
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *		the robot is in autonomous mode.
	 */
	public abstract void update(TeleopInput input);

	/**
	 * Decide the next state to transition to. This is a function of the inputs
	 * and the current state of this FSM. This method should not have any side
	 * effects on outputs. In other words, this method should only read or get
	 * values to decide what state to go to.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *		the robot is in autonomous mode.
	 * @return FSM state for the next iteration
	 */
	protected abstract S nextState(TeleopInput input);

}
