package frc.robot.systems;

import frc.robot.TeleopInput;

enum States {
    IDLE,
    REV,
    ALIGN,
    FEED
}

public class TestFSMSystem extends FSMSystem<States> {

    @Override
    public void reset() {
        setCurrentState(States.IDLE);
    }

    @Override
    public void update(TeleopInput input) {
        System.out.printf("current state: %s\n", getCurrentState());
        System.out.printf("wanted state: %s\n", getWantedState());
        setCurrentState(nextState(input));
    }

    @Override
    protected States nextState(TeleopInput input) {
        switch (getCurrentState()) {
            case IDLE:
                if (getWantedState() == null) {
                    return States.IDLE;
                }
                return switch (getWantedState()) {
                    case FEED -> States.REV;
                    case ALIGN -> States.REV;
                    case IDLE -> States.REV;
                    default -> States.IDLE;
                };
            
            case REV:
                if (getWantedState() == null) {
                    return States.REV;
                }
                return switch (getWantedState()) {
                    case FEED -> States.ALIGN;
                    case ALIGN -> States.ALIGN;
                    case IDLE -> States.IDLE;
                    default -> States.REV;
                };
            
            case ALIGN:
                return States.FEED;
            
            case FEED:
                return States.IDLE;
            
            default:
                throw new IllegalStateException();
        }
    }
    
}
