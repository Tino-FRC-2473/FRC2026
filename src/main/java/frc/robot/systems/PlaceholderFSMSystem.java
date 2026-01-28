package frc.robot.systems;

import frc.robot.input.Input;

/**
 * this is intended to be used in place of an FSM when the hardware is not present.
 */
public class PlaceholderFSMSystem extends FSMSystem<PlaceholderFSMSystem.IdentityStateSpace> {

	public enum IdentityStateSpace {
		STATE
	}

	@Override
	public void reset() {
		setCurrentState(IdentityStateSpace.STATE);
	}

	@Override
	public void update(Input input) { }

	@Override
	protected IdentityStateSpace nextState(Input input) {
		return IdentityStateSpace.STATE;
	}

}
