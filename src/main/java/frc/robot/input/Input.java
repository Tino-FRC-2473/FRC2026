package frc.robot.input;

import frc.robot.input.InputTypes.AxialInput;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Function;

import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import frc.robot.input.InputTypes.ButtonInput;

public abstract class Input {

	private EventLoop inputEventLoop;
	private Map<ButtonInput, BooleanEvent> buttonEvents;

	/**
	 * Constructs an Input object.
	 */
	public Input() {
		inputEventLoop = new EventLoop();
		buttonEvents = new HashMap<>();
		for (ButtonInput booleanSignal : ButtonInput.values()) {
			buttonEvents.put(booleanSignal, getButton(booleanSignal).apply(inputEventLoop));
		}
	}

	/**
	 * An updater for the robot. This should be called periodically
	 * in a relevant periodic method in robot.
	 */
	public void update() {
		inputEventLoop.poll();
	}

	/**
	 * Gets the (raw) button value for a specific button.
	 * @param key the button identifier
	 * @return the (raw) button value
	 */
	public boolean getButtonValue(ButtonInput key) {
		return buttonEvents.get(key).getAsBoolean();
	}

	/**
	 * Gets the button pressed value for a specific button.
	 * @param key the button identifier
	 * @return the button pressed value
	 */
	public boolean getButtonPressed(ButtonInput key) {
		return buttonEvents.get(key).rising().getAsBoolean();
	}

	/**
	 * Gets the button released value for a specific button.
	 * @param key the button identifier
	 * @return the button released value
	 */
	public boolean getButtonReleased(ButtonInput key) {
		return buttonEvents.get(key).falling().getAsBoolean();
	}

	protected abstract Function<EventLoop, BooleanEvent> getButton(ButtonInput key);

	protected BooleanEvent getBooleanEvent(ButtonInput key) {
		return buttonEvents.get(key);
	}

	/**
	 * Gets the axis value for a specific axis.
	 * @param key the axis identifier
	 * @return the axis value
	 */
	public abstract double getAxis(AxialInput key);

}
