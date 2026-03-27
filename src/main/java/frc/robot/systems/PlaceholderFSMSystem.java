package frc.robot.systems;

import frc.robot.input.Input;

/**
 * this is intended to be used in place of an FSM when the hardware is not present.
 * @param <StateSpace> the state space to mirror
 */
public class PlaceholderFSMSystem<StateSpace> extends FSMSystem<StateSpace> {

	@Override
	public void update(Input input) { }

	@Override
	protected StateSpace nextState(Input input) {
		return null;
	}

	@Override
	public void reset() { }

}
