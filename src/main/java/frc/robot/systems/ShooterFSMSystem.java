package frc.robot.systems;

import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
// Third party Hardware Imports
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import frc.robot.Constants.ShooterConstants;
import frc.robot.HardwareMap;
import frc.robot.input.Input;
// Robot Imports
import frc.robot.input.TeleopInput;
import frc.robot.motors.TalonFXWrapper;
// import frc.robot.input.InputTypes.AxialInput;
import frc.robot.input.InputTypes.ButtonInput;




public class ShooterFSMSystem extends FSMSystem<ShooterFSMSystem.ShooterFSMState> {
	enum ShooterFSMState {
		IDLE_STATE,
		SHOOTER_PREP_STATE,
		PASSER_PREP_STATE,
		INTAKE_STATE,
		MANUAL_PREP_STATE,
	}
	/* ======================== Constants ======================== */


	/* ======================== Private variables ======================== */

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	//for curPose, we need to find the height of the shooter from the floor for calculations.
	private Pose2d curPose;
	private Pose2d outpostPose;
	private Pose2d hubPose;
	private Pose2d target3Pose; //probably going to be the mirrored side of the outpost
	private TalonFX flywheelMotor;
	private TalonFX indexMotor;
	private TalonFX hoodMotor;
	private double flywheelSpeed;
	private double flywheelTargetSpeed;
	private double hoodAngle;
	private double hoodTargetAngle;
	private ShooterFSMState pastState;
	private TalonFXConfiguration hoodConfigs;
	private TalonFXConfiguration flywheelConfigs;
	private TalonFXConfiguration indexConfigs;
	private Drivetrain drivetrain;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public ShooterFSMSystem() {
		curPose = new Pose2d();
		outpostPose = ShooterConstants.OUTPOST_POSE;
		target3Pose = ShooterConstants.TARGET3_POSE;
		hubPose = ShooterConstants.HUB_POSE;

		flywheelMotor = new TalonFXWrapper(
			HardwareMap.CAN_ID_FLYWHEEL
		);
		indexMotor = new TalonFXWrapper(
			HardwareMap.CAN_ID_INDEXER
		);
		hoodMotor = new TalonFXWrapper(
			HardwareMap.CAN_ID_HOOD
		);

		hoodConfigs = new TalonFXConfiguration();

		var hoodLimitSwitchConfigs = hoodConfigs.SoftwareLimitSwitch;
		hoodLimitSwitchConfigs.ForwardSoftLimitEnable = true;
		hoodLimitSwitchConfigs.ForwardSoftLimitThreshold = ShooterConstants.HOOD_MAX_ANGLE;
		hoodLimitSwitchConfigs.ReverseSoftLimitEnable = true;
		hoodLimitSwitchConfigs.ReverseSoftLimitThreshold = ShooterConstants.HOOD_MIN_ANGLE;

		var hood0Config = hoodConfigs.Slot0;
		hood0Config.GravityType = GravityTypeValue.Arm_Cosine;
		//voltage output to overcome gravity
		hood0Config.kG = ShooterConstants.HOOD_MM_CONSTANT_G;
		//voltage output to overcome static friction
		hood0Config.kS = ShooterConstants.HOOD_MM_CONSTANT_S;
		//voltage for 1 rps in the motor, 0.12+++
		hood0Config.kV = ShooterConstants.MM_CONSTANT_V;
		//voltage for acceleration for 1 rps in the motor
		hood0Config.kA = ShooterConstants.MM_CONSTANT_A;
		//account for position error of 1 rotation
		hood0Config.kP = ShooterConstants.HOOD_MM_CONSTANT_P;
		//output for integrated error
		hood0Config.kI = ShooterConstants.HOOD_MM_CONSTANT_I;
		//account for velocity error of 1rps
		hood0Config.kD = ShooterConstants.HOOD_MM_CONSTANT_D;

		var hoodMotionMagicConfigs = hoodConfigs.MotionMagic;
		hoodMotionMagicConfigs.MotionMagicCruiseVelocity = ShooterConstants.HOOD_VELOCITY;
		hoodMotionMagicConfigs.MotionMagicAcceleration = ShooterConstants.HOOD_ACCELERATION;
		hoodMotionMagicConfigs.MotionMagicJerk = ShooterConstants.HOOD_JERK;

		var hoodFeedbackConfigs = hoodConfigs.Feedback;
		//set to 2 (divided by 360 to get in terms of degrees)
		var hoodRatio = ShooterConstants.HOOD_GEAR_RATIO / ShooterConstants.FLYWHEEL_MAX_DEGREES;
		hoodFeedbackConfigs.SensorToMechanismRatio = hoodRatio;

		hoodMotor.getConfigurator().apply(hoodConfigs);

		flywheelConfigs = new TalonFXConfiguration();
		var flywheel0Config = flywheelConfigs.Slot0;
		//voltage output to overcome static friction
		flywheel0Config.kS = ShooterConstants.FLYWHEEL_MM_CONSTANT_S;
		//voltage for 1 rps in the motor, 0.12
		flywheel0Config.kV = ShooterConstants.MM_CONSTANT_V;
		//voltage for acceleration for 1 rps in the motor
		flywheel0Config.kA = ShooterConstants.MM_CONSTANT_A;
		//account for position error of 1 rotation
		flywheel0Config.kP = ShooterConstants.FLYWHEEL_MM_CONSTANT_P;
		//output for integrated error
		flywheel0Config.kI = ShooterConstants.FLYWHEEL_MM_CONSTANT_I;
		//account for velocity error of 1rps
		flywheel0Config.kD = ShooterConstants.FLYWHEEL_MM_CONSTANT_D;

		var flywheelMotionMagicConfigs = flywheelConfigs.MotionMagic;
		//160 rps/s
		flywheelMotionMagicConfigs.MotionMagicAcceleration = ShooterConstants.FLYWHEEL_ACCELERATION;
		//1600 rps/s/s, 10* acceleration
		flywheelMotionMagicConfigs.MotionMagicJerk = ShooterConstants.FLYWHEEL_JERK;

		var flywheelFeedbackConfigs = flywheelConfigs.Feedback;
		//set to 3
		flywheelFeedbackConfigs.SensorToMechanismRatio = ShooterConstants.FLYWHEEL_GEAR_RATIO;

		flywheelMotor.getConfigurator().apply(flywheelConfigs);

		indexConfigs = new TalonFXConfiguration();
		var indexFeedbackConfigs = indexConfigs.Feedback;
		//set to 3
		indexFeedbackConfigs.SensorToMechanismRatio = ShooterConstants.INDEXER_GEAR_RATIO;

		indexMotor.getConfigurator().apply(indexConfigs);

		hoodMotor.setPosition(ShooterConstants.HOOD_MAX_ANGLE);

		BaseStatusSignal.setUpdateFrequencyForAll(
				ShooterConstants.UPDATE_FREQUENCY_HZ,
				hoodMotor.getPosition(),
				hoodMotor.getVelocity(),
				hoodMotor.getAcceleration(),
				hoodMotor.getMotorVoltage(),
				hoodMotor.getRotorPosition(),
				hoodMotor.getRotorVelocity()
		);

		BaseStatusSignal.setUpdateFrequencyForAll(
				ShooterConstants.UPDATE_FREQUENCY_HZ,
				indexMotor.getPosition(),
				indexMotor.getVelocity(),
				indexMotor.getAcceleration(),
				indexMotor.getMotorVoltage(),
				indexMotor.getRotorPosition(),
				indexMotor.getRotorVelocity()
		);

		BaseStatusSignal.setUpdateFrequencyForAll(
				ShooterConstants.UPDATE_FREQUENCY_HZ,
				flywheelMotor.getPosition(),
				flywheelMotor.getVelocity(),
				flywheelMotor.getAcceleration(),
				flywheelMotor.getMotorVoltage(),
				flywheelMotor.getRotorPosition(),
				flywheelMotor.getRotorVelocity()
		);

		hoodMotor.optimizeBusUtilization();
		indexMotor.optimizeBusUtilization();
		flywheelMotor.optimizeBusUtilization();
		reset();
	}
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots. This also
	 * passes in the drivetrain to continuously update poses for shooter_prep
	 * and passer_prep.
	 * @param driveSystem The drive system to be used by our bot
	 */
	public ShooterFSMSystem(Drivetrain driveSystem) {
		// Perform hardware init using a wrapper class
		// this is so we can see motor outputs during simulatiuons
		this();
		drivetrain = driveSystem;
		curPose = drivetrain.getPose();
	}

	/* ======================== Public methods ======================== */

	// overridden methods don't require javadocs
	// however, you may want to add implementation specific javadocs
	/**
	 * Checks if the target flywheel speed matches the actual flywheel speed within margin of error.
	 * @return Boolean statement whether or not it is at flywheel speed or not
	 */
	public boolean isAtSpeed() {
		return (Math.abs(flywheelTargetSpeed - flywheelSpeed) <= ShooterConstants.FLYWHEEL_MOE);
	}

	/**
	 * Checks if the target angle matches the actual angle within a margin of error.
	 * @return Boolean statement whether or not it is at angle or not
	 */
	public boolean isAtAngle() {
		return (Math.abs(hoodTargetAngle - hoodAngle) < ShooterConstants.HOOD_MOE);
	}

	@Override
	public void reset() {
		setCurrentState(ShooterFSMState.IDLE_STATE);

		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}

	@Override
	public void update(Input input) {
		curPose = drivetrain.getPose();
		switch (getCurrentState()) {
			case IDLE_STATE:
				handleIdleState((TeleopInput) input);
				break;

			case SHOOTER_PREP_STATE:
				handleShooterPrepState((TeleopInput) input);
				break;

			case PASSER_PREP_STATE:
				handlePasserPrepState((TeleopInput) input);
				break;

			case INTAKE_STATE:
				handleIntakeState((TeleopInput) input);
				break;

			case MANUAL_PREP_STATE:
				handleManualPrepState((TeleopInput) input);
				break;

			default:
				throw new IllegalStateException("Invalid state: " + getCurrentState().toString());
		}
		setCurrentState(nextState(input));
	}

	// @Override
	// public boolean updateAutonomous(AutoFSMState autoState) {
	// 	switch (autoState) {
	// 		case STATE1:
	// 			return handleAutoState1();
	// 		case STATE2:
	// 			return handleAutoState2();
	// 		case STATE3:
	// 			return handleAutoState3();
	// 		default:
	// 			return true;
	// 	}
	// }

	/* ======================== Protected methods ======================== */

	@Override
	protected ShooterFSMState nextState(Input input) {
		switch (getCurrentState()) {
			case IDLE_STATE:
				if (input.getButtonPressed(ButtonInput.PASSER_PREP_TOGGLE)) {
					pastState = getCurrentState();
					return ShooterFSMState.PASSER_PREP_STATE;
				} else if (input.getButtonPressed(ButtonInput.SHOOTER_PREP_TOGGLE)) {
					pastState = getCurrentState();
					return ShooterFSMState.SHOOTER_PREP_STATE;
				} else if (input.getButtonPressed(ButtonInput.MANUAL_SHOOT_TOGGLE)) {
					pastState = getCurrentState();
					return ShooterFSMState.MANUAL_PREP_STATE;
				}

			case PASSER_PREP_STATE:
				if (input.getButtonPressed(ButtonInput.IDLE_SHOOTER_TOGGLE)) {
					pastState = getCurrentState();
					return ShooterFSMState.IDLE_STATE;
				}
				if (input.getButtonPressed(ButtonInput.SHOOTER_PREP_TOGGLE)) {
					pastState = getCurrentState();
					return ShooterFSMState.SHOOTER_PREP_STATE;
				}
				if (input.getButtonPressed(ButtonInput.MANUAL_SHOOT_TOGGLE)) {
					pastState = getCurrentState();
					return ShooterFSMState.MANUAL_PREP_STATE;
				}
				if (isAtSpeed() && isAtAngle() && input.getButtonPressed(ButtonInput.REV_INDEXER)) {
					//need to make sure to change colors for if its at speed and at angle so that
					// they know when to pull triggers
					pastState = getCurrentState();
					return ShooterFSMState.INTAKE_STATE;
				}

			case INTAKE_STATE:
				boolean condition = !isAtSpeed() || !isAtAngle();		
				if (!isAtSpeed() || !isAtAngle() || !input.getButtonPressed(ButtonInput.REV_INDEXER)) {
					indexMotor.set(0);
					return pastState;
					//pastState should only store shooter_prep, passer_prep, and manual_prep
				}

				if (input.getButtonPressed(ButtonInput.IDLE_SHOOTER_TOGGLE)) {
					pastState = getCurrentState();
					return ShooterFSMState.IDLE_STATE;
				}

				if (input.getButtonPressed(ButtonInput.PASSER_PREP_TOGGLE)) {
					pastState = getCurrentState();
					return ShooterFSMState.PASSER_PREP_STATE;
				}

				if (input.getButtonPressed(ButtonInput.SHOOTER_PREP_TOGGLE)) {
					pastState = getCurrentState();
					return ShooterFSMState.SHOOTER_PREP_STATE;
				}

				if (input.getButtonPressed(ButtonInput.MANUAL_SHOOT_TOGGLE)) {
					pastState = getCurrentState();
					return ShooterFSMState.MANUAL_PREP_STATE;
				}

			case SHOOTER_PREP_STATE:
				if (input.getButtonPressed(ButtonInput.IDLE_SHOOTER_TOGGLE)) {
					pastState = getCurrentState();
					return ShooterFSMState.IDLE_STATE;
				}
				if (input.getButtonPressed(ButtonInput.PASSER_PREP_TOGGLE)) {
					pastState = getCurrentState();
					return ShooterFSMState.PASSER_PREP_STATE;
				}
				if (input.getButtonPressed(ButtonInput.MANUAL_SHOOT_TOGGLE)) {
					pastState = getCurrentState();
					return ShooterFSMState.MANUAL_PREP_STATE;
				}
				if (isAtSpeed() && isAtAngle() && input.getButtonPressed(ButtonInput.REV_INDEXER)) {
					pastState = getCurrentState();
					return ShooterFSMState.INTAKE_STATE;
				}

			case MANUAL_PREP_STATE:
				// Manual can only go to idle (we need the button inputs for right and left
				// bumper to adjust manually)
				if (input.getButtonPressed(ButtonInput.IDLE_SHOOTER_TOGGLE)) {
					pastState = getCurrentState();
					return ShooterFSMState.IDLE_STATE;
				}

				boolean atTarget = isAtSpeed() && isAtAngle();
				if (atTarget && input.getButtonPressed(ButtonInput.REV_INDEXER)) {
					pastState = getCurrentState();
					return ShooterFSMState.INTAKE_STATE;
				}

			default:
				throw new IllegalStateException("Invalid state: " + getCurrentState().toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in IDLE_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *		the robot is in autonomous mode.
	 */
	private void handleIdleState(TeleopInput input) {
		flywheelTargetSpeed = 0;
		hoodTargetAngle = ShooterConstants.HOOD_MAX_ANGLE;
		updateFlywheel();
		updateHood();
		indexMotor.set(0);
		//set hoodMotor to 20 degrees/base angle?
		//hood remains at current angle.
	}
	/**
	 * Handle behavior in PASSER_PREP_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *		the robot is in autonomous mode.
	 */
	private void handlePasserPrepState(TeleopInput input) {
		Pose2d correctTarget = new Pose2d();

		double outpostDistance = (double) curPose.getTranslation()
				.getDistance(outpostPose.getTranslation());
		double target3Distance = (double) curPose.getTranslation()
				.getDistance(target3Pose.getTranslation());
		if (outpostDistance < target3Distance) {
			correctTarget = target3Pose;
		} else {
			correctTarget = outpostPose;
		}

		//index 0 is flywheel velocity, index 1 is hood angle
		List<Object> targetValues = calculateTargetValues(correctTarget);
		flywheelTargetSpeed = (double) targetValues.get(0);
		hoodTargetAngle = (double) targetValues.get(1);

		updateFlywheel();
		updateHood();
		// TBD: code to find the distance vector from where we are to passing targets
		// (preferably outpost and thelocation of outpost on the other side) (3d vector)
	}
	/**
	 * Calculate needed values to shoot in specific targets.
	 * @param target The pose we are targetting towards
	 * @return A list holding the target's needed flywheel speed and hood angle in that
	 * order to make a pass
	 */
	public List<Object> calculateTargetValues(Pose2d target) {
		return null;
		//code to be determined based off of regression model
	}

	/**
	 * Handle behavior in SHOOTER_PREP_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *		the robot is in autonomous mode.
	 */
	private void handleShooterPrepState(TeleopInput input) {
		List<Object> targetValues = calculateTargetValues(hubPose);
		flywheelTargetSpeed = (double) targetValues.get(0);
		hoodTargetAngle = (double) targetValues.get(1);

		updateFlywheel();
		//TBD: code to find the distance vector from where we are to hub center (3d vector)
	}

	/**
	 * Handle behavior in INTAKE_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *		the robot is in autonomous mode.
	 */
	private void handleIntakeState(TeleopInput input) {
		indexMotor.setVoltage(flywheelMotor.getMotorVoltage().getValueAsDouble());
	}

	/**
	 * Handle behavior in MANUAL_PREP_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *		the robot is in autonomous mode.
	 */
	private void handleManualPrepState(TeleopInput input) {

		// FOR MANUAL ONLY: Right Bumper will be used as a deincrementer. Do not confuse this
		// with triggering Passer Prep.
		// FOR MANUAL ONLY: Left Bumper will be used to adjust hood angle. Do not confuse this
		// with triggering Shooter Prep.
		//how much the hood angle increases/decreases each click

		//shooter_prep_toggle will be for hood movement
		// passer_prep_toggle will be the deincrementer
		// manual_shoot_toggle will be the flywheel control
		double hoodIncrement = ShooterConstants.HOOD_INCREMENTER;
		// for checkstyles
		boolean hoodSet = input.getButtonPressed(ButtonInput.SHOOTER_PREP_TOGGLE);
		boolean incrementSet = input.getButtonPressed(ButtonInput.PASSER_PREP_TOGGLE);
		boolean flywheelSet = input.getButtonPressed(ButtonInput.MANUAL_SHOOT_TOGGLE);

		if (hoodSet && incrementSet) {
			if (hoodTargetAngle - hoodIncrement >= ShooterConstants.HOOD_MIN_ANGLE) {
				hoodTargetAngle -= hoodIncrement;
			} else {
				hoodTargetAngle = ShooterConstants.HOOD_MIN_ANGLE;
			}
			//decrease hood angle by 5 degrees
		} else if (hoodSet) {
			if (hoodTargetAngle + hoodIncrement <= ShooterConstants.HOOD_MAX_ANGLE) {
				hoodTargetAngle += hoodIncrement;
			} else {
				hoodTargetAngle = ShooterConstants.HOOD_MAX_ANGLE;
			}
			//increase hood angle by 5 degrees
		}
		updateHood();
		double flyIncrement = ShooterConstants.FLYWHEEL_INCREMENTER;
		//how much the flywheel speed increases/decreases each click
		if (flywheelSet && incrementSet) {
			if (flywheelTargetSpeed - flyIncrement > 0) {
				flywheelTargetSpeed -= flyIncrement;
			} else {
				flywheelTargetSpeed = 0;
			}
			//decrease flywheel speed by some constant, right now set to 10 m/s
		} else if (flywheelSet) {
			if (flywheelTargetSpeed + flyIncrement < ShooterConstants.FLYWHEEL_MAX_SPEED) {
				flywheelTargetSpeed += flyIncrement;
			} else {
				flywheelTargetSpeed = ShooterConstants.FLYWHEEL_MAX_SPEED;
			}

			//increase flywheel speed by some constant, right now set to 10 m/s
		}

		updateFlywheel();

		// check if current speed of motors and current angle matches what we just set it to there
		// with the boolean conditions
	}

	private void updateFlywheel() {
		MotionMagicVelocityVoltage flywheelRequest = new MotionMagicVelocityVoltage(0);
		flywheelMotor.setControl(flywheelRequest.withVelocity(flywheelTargetSpeed));
		flywheelSpeed = flywheelMotor.getVelocity().getValue().in(Units.RotationsPerSecond);
	}

	private void updateHood() {
		MotionMagicVoltage hoodRequest = new MotionMagicVoltage(0);
		hoodMotor.setControl(hoodRequest.withPosition(hoodTargetAngle));
		hoodAngle = hoodMotor.getPosition().getValue().in(Units.Degrees);
	}
}
