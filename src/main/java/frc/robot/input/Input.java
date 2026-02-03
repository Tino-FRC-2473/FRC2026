package frc.robot.input;

import frc.robot.input.InputTypes.AxialInput;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Function;

import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import frc.robot.input.InputTypes.ButtonInput;

public abstract class Input {

	protected enum BooleanEventType {
		BASE(e -> e),
		RISING(e -> e.rising()),
		FALLING(e -> e.falling());

		private Function<BooleanEvent, BooleanEvent> eventTransformer;

		public Function<BooleanEvent, BooleanEvent> getTransformer() {
			return eventTransformer;
		}

		BooleanEventType(Function<BooleanEvent, BooleanEvent> eventModification) {
			eventTransformer = eventModification;
		}
	}

	private record ButtonInputDescriptor(ButtonInput key, BooleanEventType type) { }

	private final EventLoop inputEventLoop;
	private final Map<ButtonInputDescriptor, BooleanEvent> buttonEvents;
	private final BooleanEvent falseEvent;

	Input() {
		inputEventLoop = new EventLoop();
		buttonEvents = new HashMap<>();
		falseEvent = new BooleanEvent(inputEventLoop, () -> false);
	}

	/**
	 * Resets the input object, resetting all button bindings.
	 */
	public void reset() {
		buttonEvents.clear();
		inputEventLoop.clear();
		for (ButtonInput booleanSignal : ButtonInput.values()) {
			for (BooleanEventType type : BooleanEventType.values()) {
				buttonEvents.put(
					new ButtonInputDescriptor(booleanSignal, type),
					type.eventTransformer.apply(getButton(booleanSignal).apply(inputEventLoop))
				);
			}
		}
	}

	/**
	 * Update the input loop periodically.
	 */
	public void update() {
		inputEventLoop.poll();
	}

	/**
	 * Gets the axis value for a specific axis.
	 * @param key the axis identifier
	 * @return the axis value
	 */
	public abstract double getAxisValue(AxialInput key);

	/**
	 * Gets the (raw) button value for a specific button.
	 * @param key the button identifier
	 * @return the (raw) button value
	 */
	public boolean getButtonValue(ButtonInput key) {
		return getBooleanEvent(key, BooleanEventType.BASE).getAsBoolean();
	}

	/**
	 * Gets the button pressed value for a specific button.
	 * @param key the button identifier
	 * @return the button pressed value
	 */
	public boolean getButtonPressed(ButtonInput key) {
		return getBooleanEvent(key, BooleanEventType.RISING).getAsBoolean();
	}

	/**
	 * Gets the button released value for a specific button.
	 * @param key the button identifier
	 * @return the button released value
	 */
	public boolean getButtonReleased(ButtonInput key) {
		return getBooleanEvent(key, BooleanEventType.FALLING).getAsBoolean();
	}

	protected abstract Function<EventLoop, BooleanEvent> getButton(ButtonInput key);

	protected BooleanEvent getBooleanEvent(ButtonInput key, BooleanEventType type) {
		return buttonEvents.getOrDefault(new ButtonInputDescriptor(key, type), falseEvent);
	}

	/**
	 * Gets the axis value for a specific axis.
	 * @param key the axis identifier
	 * @return the axis value
	 */
	public abstract double getAxis(AxialInput key);

}
