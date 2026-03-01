package frc.robot.auto;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public final class CommandComposer {
	private boolean active = true;
	private List<Command> commandSequence = new ArrayList<>();
	private List<Command> currentCommandGroup = new ArrayList<>();

	public CommandComposer keepActiveIf(boolean condition) {
		active &= condition;
		return this;
	}

	public CommandComposer reactivate() {
		active = true;
		return this;
	}

	public CommandComposer doNext(Command command) {
		completeCommand();
		return with(command);
	}

	public CommandComposer with(Command command) {
		if (active) {
			currentCommandGroup.add(command);
		}
		return this;
	}

	public Command close() {
		completeCommand();
		return new SequentialCommandGroup(commandSequence.toArray(new Command[0]));
	}

	public void completeCommand() {
		if (currentCommandGroup.size() > 1) {
			commandSequence.add(new ParallelCommandGroup(
					currentCommandGroup.toArray(new Command[0])));
		} else if (currentCommandGroup.size() == 1) {
			commandSequence.add(currentCommandGroup.get(0));
		}

		currentCommandGroup.clear();
	}
}
