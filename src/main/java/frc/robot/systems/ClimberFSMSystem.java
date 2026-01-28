package frc.robot.systems;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import frc.robot.HardwareMap;
import frc.robot.input.TeleopInput;
import frc.robot.input.InputTypes.AxialInput;
import frc.robot.input.InputTypes.ButtonInput;


import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.ClimberConstants;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import static edu.wpi.first.units.Units.Inches;



//import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
//import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class ClimberFSMSystem  {
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

	private final TalonFX climberMotorLeft;
	private final TalonFX climberMotorRight;
	private final DigitalInput groundLimitSwitchLeft;
	private final DigitalInput groundLimitSwitchRight;

	private ElevatorSim m_sim;
	private DIOSim m_limitSimLeft;
    private DIOSim m_limitSimRight;


	private ClimberFSMState currentState;
	private MotionMagicVoltage motionRequest;

	private final double CLIMBER_ANGLE_RAD = Math.toRadians(48.0);


	/**
	 * Create ClimberFSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public ClimberFSMSystem() {
		climberMotorLeft = new TalonFX(HardwareMap.CAN_ID_CLIMBER_LEFT);
		climberMotorRight = new TalonFX(HardwareMap.CAN_ID_CLIMBER_RIGHT);
		groundLimitSwitchLeft = new DigitalInput(HardwareMap.
			CLIMBER_GROUND_LIMIT_SWITCH_DIO_PORT_LEFT);
		groundLimitSwitchRight = new DigitalInput(HardwareMap.
			CLIMBER_GROUND_LIMIT_SWITCH_DIO_PORT_RIGHT);


		climberMotorRight.setControl(new Follower(HardwareMap.CAN_ID_CLIMBER_LEFT,
			MotorAlignmentValue.Opposed));
		motionRequest = new MotionMagicVoltage(0);
		var talonFXConfigs = new TalonFXConfiguration();
		var outputConfigs = talonFXConfigs.MotorOutput;
		outputConfigs.NeutralMode = NeutralModeValue.Brake;

		var swLimitSwitch = talonFXConfigs.SoftwareLimitSwitch;
		swLimitSwitch.ForwardSoftLimitEnable = true;
		swLimitSwitch.ReverseSoftLimitEnable = true;
		swLimitSwitch.ForwardSoftLimitThreshold = ClimberConstants.UPPER_THRESHOLD.in(Inches);
		swLimitSwitch.ReverseSoftLimitThreshold = Inches.of(0).in(Inches);

		var sensorConfig = talonFXConfigs.Feedback;
		sensorConfig.SensorToMechanismRatio = ClimberConstants.ROTS_TO_INCHES;

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

		climberMotorLeft.getConfigurator().apply(talonFXConfigs);
		climberMotorRight.getConfigurator().apply(talonFXConfigs);

		climberMotorLeft.setPosition(0);
		currentState = ClimberFSMState.IDLE;
		
		if (RobotBase.isSimulation()) {
            // Adjust mass based on climber angle (Gravity component: mg * sin(theta))
            double effectiveMass = Units.lbsToKilograms(15.0) * Math.sin(CLIMBER_ANGLE_RAD);

            m_sim = new ElevatorSim(
                DCMotor.getKrakenX60(2),
                10.0,                                   // Gearing (Standard 10:1)
                effectiveMass,                          // Angled effective mass
                Units.inchesToMeters(1.0),              // Drum Radius
                0.0,                                    // Min Height
                Units.inchesToMeters(ClimberConstants.UPPER_THRESHOLD.in(Inches)), // Max Height
                true,                                   // Simulate Gravity
                0.0,                                    // Starting position
                0.01,                                   // Measurement StdDev
                0.0                                     // Starting velocity
            );

			m_limitSimLeft = new DIOSim(groundLimitSwitchLeft);
            m_limitSimRight = new DIOSim(groundLimitSwitchRight);
        }

		reset();
	}


	/**
	 * Get the current FSM state.
	 *
	 * @return current FSM state
	 */
	@AutoLogOutput(key = "Climber/State")
	public ClimberFSMState getCurrentState() {
		return currentState;
	}

	/**
	 * Reset this system to its start state. This may be called from mode init
	 * when the robot is enabled.
	 *
	 * Note this is distinct from the one-time initialization in the constructor
	 * as it may be called multiple times in a boot cycle,
	 * Ex. if the robot is enabled, disabled, then reenabled.
	 */
	public void reset() {
		currentState = ClimberFSMState.AUTO_IDLE;
		update(null);
	}

	/**
	 * Update FSM based on new inputs. This function only calls the FSM state
	 * specific handlers.
	 *
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *              the robot is in autonomous mode.
	 */
	public void update(TeleopInput input) {

		if (RobotBase.isSimulation()) {
            m_sim.setInputVoltage(climberMotorLeft.getSimState().getMotorVoltage());
            m_sim.update(0.020);

            double posInches = Units.metersToInches(m_sim.getPositionMeters());
            double velInchesPerSec = Units.metersToInches(m_sim.getVelocityMetersPerSecond());
            // Feed sim results back to motor state
            var simState = climberMotorLeft.getSimState();
            simState.setRawRotorPosition(posInches / ClimberConstants.ROTS_TO_INCHES);
            simState.setRotorVelocity(velInchesPerSec / ClimberConstants.ROTS_TO_INCHES);

            // Update limit switch sims
            boolean atBottom = posInches <= 0.1;
            m_limitSimLeft.setValue(atBottom);
            m_limitSimRight.setValue(atBottom);

        }

		if (input == null) {
			return;
		}

		switch (currentState) {
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
			
			default -> throw new IllegalStateException("Invalid state: " + currentState.toString());
		}

		currentState = nextState(input);
		updateLogging();
	}

	/**
	 * Update logging values for this system.
	 */
	public void updateLogging() {
		double currentHeight = getClimberHeightInches();
		double extension = Units.inchesToMeters(currentHeight);
        double x = extension * Math.cos(CLIMBER_ANGLE_RAD);
        double z = extension * Math.sin(CLIMBER_ANGLE_RAD);
		Logger.recordOutput("Climber/3DPose", new Pose3d(
            new Translation3d(x, 0, z), 
            new Rotation3d(0, -CLIMBER_ANGLE_RAD, 0)
        ));

		Logger.recordOutput("Climber/Control Request",
			climberMotorLeft.getAppliedControl().toString().
				substring(ClimberConstants.CONTROL_REQUEST_SUBSTRING_START_INDEX));
	}

	@AutoLogOutput(key = "Climber/Position", unit = "rotations")
	public double getMotorPosition() {
		return climberMotorLeft.getPosition().getValueAsDouble();
	}

	@AutoLogOutput(key = "Climber/Velocity", unit = "rps")
	public double getMotorVelocity() {
		return climberMotorLeft.getVelocity().getValueAsDouble();
	}

	@AutoLogOutput(key = "Climber/Applied Voltage", unit = "volts")
	public double getMotorVoltage() {
		return climberMotorLeft.getMotorVoltage().getValueAsDouble();
	}


	@AutoLogOutput(key = "Climber/Height Inches", unit = "inches")
	private double getClimberHeightInches() {
		return climberMotorLeft.getPosition().getValueAsDouble();
	}

	private boolean isOnGround() {
		double height = getClimberHeightInches();
		return (height <= 0.0 || groundLimitSwitchLeft.get() || groundLimitSwitchRight.get());
	}

	@AutoLogOutput(key = "Climber/Is Extended L1?")
	private boolean isExtendedL1() {
		double height = getClimberHeightInches();
		return height >= ClimberConstants.L1_EXTEND_POS.in(Inches)
			- ClimberConstants.POSITION_TOLERANCE_L1.in(Inches);
	}

	@AutoLogOutput(key = "Climber/Right Is At Bottom?")
	private boolean rightLimit() {
		return groundLimitSwitchRight.get();
	}

	@AutoLogOutput(key = "Climber/Left Is At Bottom?")
	private boolean leftLimit() {
		return groundLimitSwitchLeft.get();
	}

	private boolean isRetractedL1() {
		double height = getClimberHeightInches();
		return height <= ClimberConstants.L1_RETRACT_POS.in(Inches)
			+ ClimberConstants.POSITION_TOLERANCE_L1.in(Inches);
	}

	private ClimberFSMState nextState(TeleopInput input) {
		if (input == null) {
			return ClimberFSMState.IDLE;
		}

		switch (currentState) {
			case AUTO_IDLE:
				if (input.getButtonPressed(ButtonInput.CLIMBER_NEXT_STEP)) {
					return ClimberFSMState.AUTO_DOWN_1;
				}
				if (input.getButtonPressed(ButtonInput.CLIMBER_EMERGENCY_ABORT)) {
					return ClimberFSMState.IDLE;
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
				if (input.getButtonPressed(ButtonInput.CLIMBER_NEXT_STEP)) {
					return ClimberFSMState.IDLE;
				}
				return ClimberFSMState.MANUAL_DIRECT_CONTROL;
			case AUTO_UP_1:
				if (isExtendedL1()) {
					return ClimberFSMState.IDLE;
				}
				return ClimberFSMState.AUTO_UP_1;
			case AUTO_UP_2:
				if (isRetractedL1()) {
					return ClimberFSMState.IDLE;
				}
				return ClimberFSMState.AUTO_UP_2;
			case AUTO_DOWN_1:
				if (input.getButtonPressed(ButtonInput.CLIMBER_DOWN_BUTTON) && isExtendedL1()) {
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
					if (currentState == ClimberFSMState.L1_EXTEND) {
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
			default:
				throw new UnsupportedOperationException("Unknown state");
		}
	}

	private void handleIdleState(TeleopInput input) {
		climberMotorLeft.set(0);

	}

	private void handleManualDirectControlState(TeleopInput input) {
		double manualControlValue = MathUtil.applyDeadband(input
			.getAxis(AxialInput.CLIMBER_MANUAL_CONTROL),
				ClimberConstants.JOYSTICK_DEADBAND);
		if (groundLimitSwitchLeft.get() || groundLimitSwitchRight.get()) {
			climberMotorLeft.setPosition(0);
			return;
		}
		if (!((manualControlValue < 0 && getClimberHeightInches() <= Inches.of(0).in(Inches))
			|| (manualControlValue > 0 && getClimberHeightInches()
			>= ClimberConstants.UPPER_THRESHOLD.in(Inches)))) {
			climberMotorLeft.set(manualControlValue * ClimberConstants.MANUAL_SCALE);
		} else {
			climberMotorLeft.set(0);
		}
	}

	private void handleL1ExtendState(TeleopInput input) {
		climberMotorLeft.setControl(motionRequest.withPosition(
			ClimberConstants.L1_EXTEND_POS.in(Inches)
		));
	}

	private void handleL1RetractState(TeleopInput input) {
		climberMotorLeft.setControl(motionRequest.withPosition(
			ClimberConstants.L1_RETRACT_POS.in(Inches)
		));
	}

	private void handleResetToZero(TeleopInput input) {
		if (groundLimitSwitchLeft.get() || getClimberHeightInches() <= 0) {
			climberMotorLeft.set(0);
		} else {
			climberMotorLeft.setControl(motionRequest.withPosition(
				ClimberConstants.GROUND.in(Inches)
			));
		}
	}
}
