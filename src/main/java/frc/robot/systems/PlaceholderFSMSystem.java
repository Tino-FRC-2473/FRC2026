package frc.robot.systems;

import frc.robot.TeleopInput;
import frc.robot.systems.AutoHandlerSystem.AutoFSMState;

enum IdentityStateSpace {
	STATE
}

/**
 * this is intended to be used in place of an FSM when the hardware is not present.
 */
public class PlaceholderFSMSystem extends FSMSystem<IdentityStateSpace> {

	@Override
	public void reset() {
		setCurrentState(IdentityStateSpace.STATE);
	}

	@Override
	public void update(TeleopInput input) { }

	@Override
	public boolean updateAutonomous(AutoFSMState autoState) {
		return false;
	}

	@Override
	protected IdentityStateSpace nextState(TeleopInput input) {
		return IdentityStateSpace.STATE;
	}

}
