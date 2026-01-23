package frc.robot.input;

import frc.robot.input.InputTypes.DoubleSignal;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Function;

import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import frc.robot.input.InputTypes.BooleanSignal;

public abstract class Input {

	private EventLoop inputEventLoop;
	private Map<BooleanSignal, BooleanEvent> buttonEvents;

	/**
	 * Constructs an Input object.
	 */
	public Input() {
		inputEventLoop = new EventLoop();
		buttonEvents = new HashMap<>();
		for (BooleanSignal booleanSignal : BooleanSignal.values()) {
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
	public boolean getButtonValue(BooleanSignal key) {
		return buttonEvents.get(key).getAsBoolean();
	}

	/**
	 * Gets the button pressed value for a specific button.
	 * @param key the button identifier
	 * @return the button pressed value
	 */
	public boolean getButtonPressed(BooleanSignal key) {
		return buttonEvents.get(key).rising().getAsBoolean();
	}

	/**
	 * Gets the button released value for a specific button.
	 * @param key the button identifier
	 * @return the button released value
	 */
	public boolean getButtonReleased(BooleanSignal key) {
		return buttonEvents.get(key).falling().getAsBoolean();
	}

	protected abstract Function<EventLoop, BooleanEvent> getButton(BooleanSignal key);

	protected BooleanEvent getBooleanEvent(BooleanSignal key) {
		return buttonEvents.get(key);
	}

	/**
	 * Gets the axis value for a specific axis.
	 * @param key the axis identifier
	 * @return the axis value
	 */
	public abstract double getAxis(DoubleSignal key);

}
