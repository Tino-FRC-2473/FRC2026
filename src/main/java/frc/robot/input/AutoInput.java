package frc.robot.input;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Function;

import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.input.InputTypes.DoubleSignal;
import frc.robot.Robot;
import frc.robot.input.InputTypes.BooleanSignal;

public final class AutoInput extends Input {

	public static final int PULSE_DURATION_TICKS = 5;

	private Map<BooleanSignal, Boolean> buttonValues;
	private Map<DoubleSignal, Double> axesValues;

	/**
	 * Constructs an AutonInput to store input from commands
	 * for the FSMs. Uses wpi event loops to handle pressed / released / raw behavior.
	 */
	public AutoInput() {
		buttonValues = new HashMap<>();
		axesValues = new HashMap<>();
	}

	/**
	 * Setter for a button value.
	 * @param button the button
	 * @param value the value
	 */
	public void setButton(BooleanSignal button, boolean value) {
		buttonValues.put(button, value);
	}

	/**
	 * Returns an instant command that toggles the button value.
	 * @param button the button to set
	 * @return the command
	 */
	public Command toggleButtonCommand(BooleanSignal button) {
		return new InstantCommand(() -> buttonValues.put(button, buttonValues.get(button)));
	}

	/**
	 * Returns an instant command that toggles and then untoggles a button shortly after.
	 * This is intended to be used to simulates a brief button press.
	 * @param button the button to set
	 * @return the command
	 */
	public Command pulseButtonCommand(BooleanSignal button) {
		return new SequentialCommandGroup(
			toggleButtonCommand(button),
			new WaitCommand(PULSE_DURATION_TICKS * Robot.defaultPeriodSecs),
			toggleButtonCommand(button)
		);
	}

	/**
	 * Setter for an axis value.
	 * @param axis the axis
	 * @param value the value
	 */
	public void setAxis(DoubleSignal axis, double value) {
		axesValues.put(axis, value);
	}

	@Override
	public double getAxis(DoubleSignal key) {
		return axesValues.get(key);
	}

	@Override
	protected Function<EventLoop, BooleanEvent> getButton(BooleanSignal key) {
		return (e) -> new BooleanEvent(e, () -> buttonValues.get(key));
	}

}
