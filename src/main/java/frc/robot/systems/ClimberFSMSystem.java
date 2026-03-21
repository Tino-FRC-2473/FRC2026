package frc.robot.systems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.simulation.DIOSim;

import frc.robot.HardwareMap;
import frc.robot.input.Input;
import frc.robot.motors.TalonFXWrapper;
import frc.robot.util.EnergyLogger;
import frc.robot.input.InputTypes.AxialInput;
import frc.robot.input.InputTypes.ButtonInput;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.ClimberConstants;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;


public class ClimberFSMSystem extends FSMSystem<ClimberFSMSystem.ClimberFSMState> {
	public enum ClimberFSMState {
		IDLE,
		MANUAL_DIRECT_CONTROL,
		L1_EXTEND,
		L1_RETRACT,
		AUTO_UP_1,
		AUTO_UP_2,
		AUTO_DOWN_1,
		AUTO_DOWN_2,
		LOCKED_FINAL,
		AUTO_IDLE
	}

	private final TalonFXWrapper climberMotorLeft;
	private final TalonFXWrapper climberMotorRight;
	private final DigitalInput groundLimitSwitchLeft;
	private final DigitalInput groundLimitSwitchRight;

	private ElevatorSim sim;
	private DIOSim limitSimLeft;
	private DIOSim limitSimRight;
	private MotionMagicVoltage motionRequest;
	private Optional<IntakeFSMSystem> intake;

	/**
	 * Create ClimberFSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 * @param intakeFSMSystem the IntakeFSMSystem
	 */
	public ClimberFSMSystem(Optional<IntakeFSMSystem> intakeFSMSystem) {
		intake = intakeFSMSystem;
		climberMotorLeft = new TalonFXWrapper(HardwareMap.CAN_ID_CLIMBER_LEFT);
		climberMotorRight = new TalonFXWrapper(HardwareMap.CAN_ID_CLIMBER_RIGHT);
		climberMotorRight.setControl(new Follower(HardwareMap.CAN_ID_CLIMBER_LEFT,
			MotorAlignmentValue.Opposed));


		motionRequest = new MotionMagicVoltage(0);
		var talonFXConfigs = getConfig();
		climberMotorLeft.getConfigurator().apply(talonFXConfigs);
		climberMotorRight.getConfigurator().apply(talonFXConfigs);
		climberMotorRight.setPosition(0);
		climberMotorLeft.setPosition(0);


		groundLimitSwitchLeft = new DigitalInput(HardwareMap.
			CLIMBER_GROUND_LIMIT_SWITCH_DIO_PORT_LEFT);
		groundLimitSwitchRight = new DigitalInput(HardwareMap.
			CLIMBER_GROUND_LIMIT_SWITCH_DIO_PORT_RIGHT);


		if (RobotBase.isSimulation()) {

			double effectiveMass = ClimberConstants.EFFECTIVE_WEIGHT;
			sim = new ElevatorSim(
				DCMotor.getKrakenX60(2),
				ClimberConstants.CLIMBER_GEAR_RATIO,
				effectiveMass,
				Units.inchesToMeters(1.0),
				0.0,
				Units.inchesToMeters(ClimberConstants.UPPER_THRESHOLD.in(Inches)),
				true,
				0.0,
				0.0,
				0.0
			);

			limitSimLeft = new DIOSim(groundLimitSwitchLeft);
			limitSimRight = new DIOSim(groundLimitSwitchRight);
		}
		climberMotorLeft.setPosition(0);
		setCurrentState(ClimberFSMState.AUTO_IDLE);

		reset();
	}


	/**
	 * Get the TalonFXConfiguration for the climber motors.
	 * @return TalonFXConfiguration for the climber motors
	 */
	public TalonFXConfiguration getConfig() {
		var talonFXConfigs = new TalonFXConfiguration();

		var outputConfigs = talonFXConfigs.MotorOutput;
		outputConfigs.NeutralMode = NeutralModeValue.Brake;
		outputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

		var swLimitSwitch = talonFXConfigs.SoftwareLimitSwitch;
		swLimitSwitch.ForwardSoftLimitEnable = true;
		swLimitSwitch.ReverseSoftLimitEnable = true;
		swLimitSwitch.ForwardSoftLimitThreshold = ClimberConstants.UPPER_THRESHOLD.in(Inches);
		swLimitSwitch.ReverseSoftLimitThreshold = ClimberConstants.
			REVERSE_LIMIT_SWITCH_POS.in(Inches);

		var sensorConfig = talonFXConfigs.Feedback;
		sensorConfig.SensorToMechanismRatio = 1 / ClimberConstants.ROTS_TO_INCHES;

		var slot0 = talonFXConfigs.Slot0;
		slot0.GravityType = GravityTypeValue.Elevator_Static;
		slot0.kG = ClimberConstants.KG;
		slot0.kS = ClimberConstants.KS;
		slot0.kV = ClimberConstants.KV;
		slot0.kA = ClimberConstants.KA;
		slot0.kP = ClimberConstants.KP;
		slot0.kI = ClimberConstants.KI;
		slot0.kD = ClimberConstants.KD;
		slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

		var motionMagicConfigs = talonFXConfigs.MotionMagic;
		motionMagicConfigs.MotionMagicCruiseVelocity = ClimberConstants.CRUISE_VELO;
		motionMagicConfigs.MotionMagicAcceleration = ClimberConstants.TARGET_ACCEL;
		motionMagicConfigs.MotionMagicExpo_kV = ClimberConstants.EXPO_KV;

		return talonFXConfigs;
	}

	/**
	 * Reset this system to its start state. This may be called from mode init
	 * when the robot is enabled.
	 *
	 * Note this is distinct from the one-time initialization in the constructor
	 * as it may be called multiple times in a boot cycle,
	 * Ex. if the robot is enabled, disabled, then reenabled.
	 */
	@Override
	public void reset() {
		setCurrentState(ClimberFSMState.AUTO_IDLE);
		update(null);
	}

	/**
	 * Update FSM based on new inputs. This function only calls the FSM state
	 * specific handlers.
	 *
	 * @param input Global Input if robot in teleop mode or null if
	 *			  the robot is in autonomous mode.
	 */
	@Override
	public void update(Input input) {

		boolean atBottom2 = (getLeftLimitSwitch() || getRightLimitSwitch());
		if (atBottom2) {
			climberMotorLeft.setPosition(Angle.ofBaseUnits(0, Degrees));
			climberMotorRight.setPosition(Angle.ofBaseUnits(0, Degrees));
		}

		// if (RobotBase.isSimulation()) {
		// 	sim.setInputVoltage(climberMotorLeft.getSimState().getMotorVoltage());
		// 	sim.update(ClimberConstants.UPDATE_RATE);

		// 	double drumCircumferenceMeters = ClimberConstants.DRUM_CIRCUMFERENCE_METERS;
		// 	double gearRatio = ClimberConstants.CLIMBER_GEAR_RATIO;

		// 	double rotorRotations = (sim.getPositionMeters()
		// 		/ drumCircumferenceMeters) * gearRatio;
		// 	double rotorVelocityRPS = (sim.getVelocityMetersPerSecond()
		// 		/ drumCircumferenceMeters) * gearRatio;

		// 	var simState = climberMotorLeft.getSimState();
		// 	simState.setRawRotorPosition(rotorRotations);
		// 	simState.setRotorVelocity(rotorVelocityRPS);

		// 	boolean atBottom = sim.getPositionMeters() <= ClimberConstants.LIMIT_SWITCH_HEIGHT;
		// 	limitSimLeft.setValue(atBottom);
		// 	limitSimRight.setValue(atBottom);

		// }

		if (input == null) {
			return;
		}

		switch (getCurrentState()) {
			case IDLE -> handleIdleState(input);
			case AUTO_IDLE -> handleIdleState(input);
			case LOCKED_FINAL -> handleIdleState(input);
			case MANUAL_DIRECT_CONTROL -> handleManualDirectControlState(input);
			case L1_EXTEND -> handleL1ExtendState(input);
			case L1_RETRACT -> handleL1RetractState(input);
			case AUTO_DOWN_1 -> handleL1ExtendState(input);
			case AUTO_DOWN_2 -> handleResetToZero(input);
			case AUTO_UP_1 -> handleL1ExtendState(input);
			case AUTO_UP_2 -> handleL1RetractState(input);

			default -> throw new IllegalStateException(
					"Invalid state: " + getCurrentState().toString());
		}

		EnergyLogger.recordEnergyUsage("Climber", 
			climberMotorLeft.getLoggedCurrent(), 
			climberMotorRight.getLoggedCurrent());

		setCurrentState(nextState(input));
		updateLogging();
	}

	/**
	 * Update logging values for this system.
	 */
	public void updateLogging() {
		double currentHeight = getClimberHeightInches();
		double extension = Units.inchesToMeters(currentHeight);
		double x = extension * Math.cos(ClimberConstants.CLIMBER_ANGLE_RAD);
		double z = extension * Math.sin(ClimberConstants.CLIMBER_ANGLE_RAD);

		Logger.recordOutput("Climber/3DPose", new Pose3d(
			new Translation3d(x, 0, z),
			new Rotation3d(0, -ClimberConstants.CLIMBER_ANGLE_RAD, 0)
		));

		Logger.recordOutput("Climber/Control Request",
			climberMotorLeft.getAppliedControl().toString().
				substring(ClimberConstants.CONTROL_REQUEST_SUBSTRING_START_INDEX));

		Logger.recordOutput("Climber/Expected Position", motionRequest.getPositionMeasure());
	}

	/**
	 * Get the motor position in rotations.
	 *
	 * @return motor position in rotations
	 */
	@AutoLogOutput(key = "Climber/Position", unit = "rotations")
	public double getMotorPosition() {
		return climberMotorLeft.getPosition().getValueAsDouble();
	}

	/**
	 * Get the motor velocity in rotations per second.
	 *
	 * @return motor velocity in rotations per second
	 */
	@AutoLogOutput(key = "Climber/Velocity", unit = "rps")
	public double getMotorVelocity() {
		return climberMotorLeft.getVelocity().getValueAsDouble();
	}

	/**
	 * Get the Left motor voltage in volts.
	 *
	 * @return motor voltage in volts
	 */
	@AutoLogOutput(key = "Climber/Left Applied Voltage", unit = "volts")
	public double getLeftMotorVoltage() {
		return climberMotorLeft.getMotorVoltage().getValueAsDouble();
	}

	/**
	 * Get the Right motor voltage in volts.
	 *
	 * @return motor voltage in volts
	 */
	@AutoLogOutput(key = "Climber/Right Applied Voltage", unit = "volts")
	public double getRightMotorVoltage() {
		return climberMotorRight.getMotorVoltage().getValueAsDouble();
	}

	private boolean getLeftLimitSwitch() {
		return !groundLimitSwitchLeft.get();
	}

	private boolean getRightLimitSwitch() {
		return !groundLimitSwitchRight.get();
	}

	/**
	 * @return the current state
	 */
	@AutoLogOutput(key = "Climber/Current State")
	public ClimberFSMState getClimberState() {
		return getCurrentState();
	}

	/**
	 * Get the climber height in inches.
	 *
	 * @return climber height in inches
	 */
	@AutoLogOutput(key = "Climber/Height Inches", unit = "inches")
	private double getClimberHeightInches() {
		return climberMotorLeft.getPosition().getValueAsDouble();
	}

	private boolean isOnGround() {
		double height = getClimberHeightInches();
		return (height <= 0.0 || getLeftLimitSwitch() || getRightLimitSwitch());
	}

	@AutoLogOutput(key = "Climber/Is Extended L1?")
	private boolean isExtendedL1() {
		double height = getClimberHeightInches();
		return height >= ClimberConstants.L1_EXTEND_POS.in(Inches)
			- ClimberConstants.POSITION_TOLERANCE_L1.in(Inches);
	}

	/**
	 * Returns a command that ends when isExtendedL1 is true.
	 * @return a command that ends when isExtendedL1 is true
	 */
	public Command waitForExtendedL1() {
		return new Command() {
			@Override
			public boolean isFinished() {
				return super.isFinished() && isExtendedL1();
			}
		};
	}

	@AutoLogOutput(key = "Climber/Right Is At Bottom?")
	private boolean rightLimit() {
		return getRightLimitSwitch();
	}

	@AutoLogOutput(key = "Climber/Left Is At Bottom?")
	private boolean leftLimit() {
		return getLeftLimitSwitch();
	}



	private boolean isRetractedL1() {
		double height = getClimberHeightInches();
		return (height <= ClimberConstants.L1_RETRACT_POS.in(Inches)
			+ ClimberConstants.DOWN_POSITION_TOLERANCE_L1.in(Inches));
	}

	@Override
	protected ClimberFSMState nextState(Input input) {
		if (input == null) {
			return ClimberFSMState.IDLE;
		}

		switch (getCurrentState()) {
			case AUTO_IDLE:
				if (input.getButtonPressed(ButtonInput.CLIMBER_NEXT_STEP)) {
					return ClimberFSMState.AUTO_DOWN_1;
				}
				if (input.getButtonPressed(ButtonInput.CLIMBER_EMERGENCY_ABORT)) {
					return ClimberFSMState.IDLE;
				}
				if (input.getButtonPressed(ButtonInput.CLIMBER_AUTO_UP_1)) {
					return ClimberFSMState.AUTO_UP_1;
				}
				if (input.getButtonPressed(ButtonInput.CLIMBER_AUTO_UP_2)) {
					return ClimberFSMState.AUTO_UP_2;
				}
				return ClimberFSMState.AUTO_IDLE;
			case IDLE:
				if (input.getButtonPressed(ButtonInput.CLIMBER_MANUAL_OVERRIDE)) {
					return ClimberFSMState.MANUAL_DIRECT_CONTROL;
				}
				if (input.getButtonPressed(ButtonInput.CLIMBER_NEXT_STEP)) {
					return ClimberFSMState.L1_EXTEND;
				}
				return ClimberFSMState.IDLE;
			case MANUAL_DIRECT_CONTROL:
				if (input.getButtonPressed(ButtonInput.CLIMBER_EMERGENCY_ABORT)) {
					return ClimberFSMState.IDLE;
				}
				return ClimberFSMState.MANUAL_DIRECT_CONTROL;
			case AUTO_UP_1:
				if (isExtendedL1()) {
					return ClimberFSMState.AUTO_IDLE;
				}
				return ClimberFSMState.AUTO_UP_1;
			case AUTO_UP_2:
				if (isRetractedL1()) {
					return ClimberFSMState.AUTO_IDLE;
				}
				return ClimberFSMState.AUTO_UP_2;
			case AUTO_DOWN_1:
				if (input.getButtonPressed(ButtonInput.CLIMBER_NEXT_STEP) && isExtendedL1()) {
					return ClimberFSMState.AUTO_DOWN_2;
				}
				return ClimberFSMState.AUTO_DOWN_1;
			case AUTO_DOWN_2:
				if (isOnGround()) {
					return ClimberFSMState.IDLE;
				}
				return ClimberFSMState.AUTO_DOWN_2;
			case L1_EXTEND:
				if (input.getButtonPressed(ButtonInput.CLIMBER_EMERGENCY_ABORT)) {
					return ClimberFSMState.IDLE;
				}
				boolean shouldAdvanceExtendedL1 =
					(input.getButtonPressed(ButtonInput.CLIMBER_NEXT_STEP) && isExtendedL1());
				if (shouldAdvanceExtendedL1) {
					if (getCurrentState() == ClimberFSMState.L1_EXTEND) {
						return ClimberFSMState.L1_RETRACT;
					}
				}
				return ClimberFSMState.L1_EXTEND;
			case L1_RETRACT:
				if (input.getButtonPressed(ButtonInput.CLIMBER_EMERGENCY_ABORT)) {
					return ClimberFSMState.IDLE;
				}
				if (isRetractedL1()) {
					return ClimberFSMState.LOCKED_FINAL;
				}
				return ClimberFSMState.L1_RETRACT;
			case LOCKED_FINAL:
				return ClimberFSMState.LOCKED_FINAL;
			default:
				throw new UnsupportedOperationException("Unknown state");
		}
	}

	private void handleIdleState(Input input) {
		climberMotorLeft.set(0);

	}

	private void handleManualDirectControlState(Input input) {
		double manualControlValue = MathUtil.applyDeadband(input
			.getAxisValue(AxialInput.CLIMBER_MANUAL_CONTROL),
				ClimberConstants.JOYSTICK_DEADBAND);

		boolean atBottom = (getLeftLimitSwitch() || getRightLimitSwitch());
		boolean atTop = (getClimberHeightInches() >= ClimberConstants.UPPER_THRESHOLD.in(Inches));

		if (atBottom) {
			climberMotorLeft.setPosition(Angle.ofBaseUnits(0, Degrees));
			climberMotorRight.setPosition(Angle.ofBaseUnits(0, Degrees));
		}

		boolean isUnsafe = ((manualControlValue < 0 && atBottom)
			|| (manualControlValue > 0 && atTop));

		if (!isUnsafe) {
			climberMotorLeft.set(manualControlValue);
		} else {
			climberMotorLeft.set(0);
		}
	}

	private void handleL1ExtendState(Input input) {
		// DutyCycleOut n = new DutyCycleOut(0.5);
		// DutyCycleOut b = new DutyCycleOut(0.5);
		// climberMotorLeft.setControl(n);
		// climberMotorRight.setControl(b);
		if (true) {
			climberMotorLeft.setControl(motionRequest.withPosition(
				ClimberConstants.L1_EXTEND_POS.in(Inches)
			));
		}
	}

	private void handleL1RetractState(Input input) {
		if (true) {
			climberMotorLeft.setControl(motionRequest.withPosition(
				ClimberConstants.L1_RETRACT_POS.in(Inches)
			));
		}
	}

	private void handleResetToZero(Input input) {
		if (true)
		 {
			if (getLeftLimitSwitch() ||
				 getRightLimitSwitch()) {
				climberMotorLeft.set(0);
			} else {
				climberMotorLeft.setControl(motionRequest.withPosition(
					ClimberConstants.GROUND.in(Inches)
				));
			}
		}
	}

	private boolean isIntakeDown() {
		AtomicBoolean isDown = new AtomicBoolean(false);
		intake.ifPresent((i) -> isDown.set(i.isIntakeDown()));
		return isDown.get();
	}
}
