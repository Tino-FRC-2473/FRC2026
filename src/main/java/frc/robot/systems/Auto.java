package frc.robot.systems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.systems.FSMSystem.ObservedStateCommand.ExitBehavior;

public class Auto {
    

    public static Command getAuto(TestFSMSystem fsmSystem1) {
        return new SequentialCommandGroup(
            fsmSystem1.getStateProcessCommand(States.FEED, States.IDLE, ExitBehavior.END_ON_ENTRY)
        );
    }
}
