package frc.robot.systems;

import org.littletonrobotics.junction.Logger;
// Third party Hardware Imports
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;



import frc.robot.Constants.ShooterConstants;
import frc.robot.HardwareMap;
import frc.robot.input.Input;
// Robot Imports
import frc.robot.motors.TalonFXWrapper;
// import frc.robot.input.InputTypes.AxialInput;
import frc.robot.input.InputTypes.ButtonInput;




public class ShooterFSMSystem extends FSMSystem<ShooterFSMSystem.ShooterFSMState> {
	public enum ShooterFSMState {
		IDLE_STATE,
		SHOOTER_PREP_STATE,
		PASSER_PREP_STATE,
		FEED_STATE,
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
	private TalonFXWrapper flywheelMotor;
	private TalonFXWrapper feederMotor;
	//private SparkMax spindexMotor;
	private Measure<AngularVelocityUnit> flywheelSpeed; //Units.RotationsPerSecond
	private Measure<AngularVelocityUnit> flywheelTargetSpeed; //Units.RotationsPerSecond
	private Measure<AngleUnit> hoodAngle; //Units.Degrees
	private ShooterFSMState pastState;
	private TalonFXConfiguration flywheelConfigs;
	private TalonFXConfiguration feederConfigs;
	private Drivetrain drivetrain;
	private MotionMagicVelocityVoltage flywheelRequest;
	private MotionMagicVelocityVoltage feederRequest;
	private IntakeFSMSystem intake;
	private DigitalInput breakBeam;
	private Timer feedTimer = new Timer();
	private Timer spindexTimer = new Timer();
	private boolean noFuelStored = false;
	private boolean flywheelMotorStopped = false;
	private double spindexVoltage; //Volts

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public ShooterFSMSystem() {
		//spindexTimer.start();
		//spindexVoltage = ShooterConstants.SPINDEX_CONSTANT_VOLTAGE; //Volts
		curPose = new Pose2d();
		outpostPose = ShooterConstants.OUTPOST_POSE;
		target3Pose = ShooterConstants.TARGET3_POSE;
		hubPose = ShooterConstants.HUB_POSE;
		hoodAngle = ShooterConstants.HOOD_ANGLE;

		flywheelRequest = new MotionMagicVelocityVoltage(0);
		feederRequest = new MotionMagicVelocityVoltage(0);
		flywheelMotor = new TalonFXWrapper(
			HardwareMap.CAN_ID_FLYWHEEL
		);
		feederMotor = new TalonFXWrapper(
			HardwareMap.CAN_ID_FEEDER
		);
		// spindexMotor = new SparkMax(HardwareMap.CAN_ID_SPINDEXER,
		// 	MotorType.kBrushless);

		var limitConfigs = new CurrentLimitsConfigs();

		// enable stator current limit
		limitConfigs.StatorCurrentLimit = ShooterConstants.SHOOTER_CURRENT_LIMIT;
		limitConfigs.StatorCurrentLimitEnable = true;

		flywheelConfigs = new TalonFXConfiguration();
		var flywheel0Config = flywheelConfigs.Slot0;
		//voltage output to overcome static friction
		flywheel0Config.kS = ShooterConstants.FLYWHEEL_MM_CONSTANT_S;
		//voltage for 1 rps in the motor, 0.11
		flywheel0Config.kV = ShooterConstants.MM_CONSTANT_V;
		//account for position error of 1 rotation
		flywheel0Config.kP = ShooterConstants.FLYWHEEL_MM_CONSTANT_P;
		//output for integrated error
		flywheel0Config.kI = ShooterConstants.FLYWHEEL_MM_CONSTANT_I;
		//account for velocity error of 1rps
		flywheel0Config.kD = ShooterConstants.FLYWHEEL_MM_CONSTANT_D;

		var flywheelMotionMagicConfigs = flywheelConfigs.MotionMagic;
		//160 rps/s
		flywheelMotionMagicConfigs.MotionMagicAcceleration =
			ShooterConstants.MAGIC_ACCELERATION.in(RotationsPerSecondPerSecond);
		//1600 rps/s/s, 10* acceleration
		flywheelMotionMagicConfigs.MotionMagicJerk = ShooterConstants.MAGIC_JERK;

		var flywheelFeedbackConfigs = flywheelConfigs.Feedback;
		//set to 3
		flywheelFeedbackConfigs.SensorToMechanismRatio = ShooterConstants.FLYWHEEL_GEAR_RATIO;



		feederConfigs = new TalonFXConfiguration();
		var feeder0Config = feederConfigs.Slot0;
		feeder0Config.kS = ShooterConstants.FEEDER_MM_CONSTANT_S;
		feeder0Config.kV = ShooterConstants.MM_CONSTANT_V;
		feeder0Config.kP = ShooterConstants.FEEDER_MM_CONSTANT_P;
		feeder0Config.kI = ShooterConstants.FEEDER_MM_CONSTANT_I;
		feeder0Config.kD = ShooterConstants.FEEDER_MM_CONSTANT_D;

		var flywheelMotorOutputs = flywheelConfigs.MotorOutput;
		flywheelMotorOutputs.NeutralMode = NeutralModeValue.Coast;
		flywheelMotorOutputs.Inverted = InvertedValue.CounterClockwise_Positive;

		flywheelMotor.getConfigurator().apply(flywheelConfigs);
		flywheelMotor.getConfigurator().apply(limitConfigs);

		var feederMotionMagicConfigs = feederConfigs.MotionMagic;
		feederMotionMagicConfigs.MotionMagicAcceleration =
			ShooterConstants.MAGIC_ACCELERATION.in(RotationsPerSecondPerSecond);
		feederMotionMagicConfigs.MotionMagicJerk = ShooterConstants.MAGIC_JERK;
		var feederFeedbackConfigs = feederConfigs.Feedback;
		//set to 3
		feederFeedbackConfigs.SensorToMechanismRatio = ShooterConstants.FEEDER_GEAR_RATIO;
		var feederMotorConfigs = feederConfigs.MotorOutput;
		feederMotorConfigs.Inverted = InvertedValue.Clockwise_Positive;

		feederMotor.getConfigurator().apply(feederConfigs);
		feederMotor.getConfigurator().apply(limitConfigs);

		BaseStatusSignal.setUpdateFrequencyForAll(
				ShooterConstants.UPDATE_FREQUENCY_HZ,
				feederMotor.getPosition(),
				feederMotor.getVelocity(),
				feederMotor.getAcceleration(),
				feederMotor.getMotorVoltage(),
				feederMotor.getRotorPosition(),
				feederMotor.getRotorVelocity()
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
		feederMotor.optimizeBusUtilization();
		flywheelMotor.optimizeBusUtilization();

		breakBeam = new DigitalInput(HardwareMap.STORAGE_BREAK_BEAM_DIO_PORT);
		reset();
	}
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots. This also
	 * passes in the drivetrain to continuously update poses for shooter_prep
	 * and passer_prep.
	 * @param driveSystem The drive system to be used by our bot
	 * @param intakeSystem The intake system to be used by our bot
	 */
	public ShooterFSMSystem(Drivetrain driveSystem, IntakeFSMSystem intakeSystem) {
		// Perform hardware init using a wrapper class
		// this is so we can see motor outputs during simulatiuons
		this();
		drivetrain = driveSystem;
		curPose = drivetrain.getPose();
		intake = intakeSystem;
	}

	/* ======================== Public methods ======================== */

	// overridden methods don't require javadocs
	// however, you may want to add implementation specific javadocs
	/**
	 * Checks if the target flywheel speed matches the actual flywheel speed within margin of error.
	 * @return Boolean statement whether or not it is at flywheel speed or not
	 */
	public boolean isAtSpeed() {

		double flyDifference =
			flywheelTargetSpeed.in(RotationsPerSecond) - flywheelSpeed.in(RotationsPerSecond);
		return (
			Math.abs(flyDifference) <= ShooterConstants.FLYWHEEL_MOE.in(RotationsPerSecond)
			);
	}

	// /**
	// * Checks if the target angle matches the actual angle within a margin of error.
	// * @return Boolean statement whether or not it is at angle or not
	// */
	// public boolean isAtAngle() {
	// 	double hoodDifference = hoodTargetAngle.in(Degrees) - hoodAngle.in(Degrees);
	// 	return (
	// 		Math.abs(hoodDifference) < ShooterConstants.HOOD_MOE.in(Degrees)
	// 		);
	// }

	@Override
	public void reset() {
		feedTimer.reset();
		feedTimer.stop();
		setCurrentState(ShooterFSMState.IDLE_STATE);

		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}

	@Override
	public void update(Input input) {
		if (drivetrain != null) {
			curPose = drivetrain.getPose();
		}
		if (getCurrentState() != null) {
			switch (getCurrentState()) {
				case IDLE_STATE:
					handleIdleState(input);
					break;

				case SHOOTER_PREP_STATE:
					handleShooterPrepState(input);
					break;

				case PASSER_PREP_STATE:
					handlePasserPrepState(input);
					break;

				case FEED_STATE:
					handleFeedState(input);
					break;

				case MANUAL_PREP_STATE:
					handleManualPrepState(input);
					break;

				default:
					throw new IllegalStateException("Invalid state: "
					+ getCurrentState().toString());
			}
		}
		Logger.recordOutput("Shooter State", getCurrentState());
		if (flywheelMotor.getVelocity() != null) {
			double curSpeed = flywheelMotor.getVelocity().getValue().in(RotationsPerSecond);
			flywheelSpeed = RotationsPerSecond.of(curSpeed * ShooterConstants.FLYWHEEL_GEAR_RATIO);
			Logger.recordOutput("Actual Motor Speed", flywheelSpeed.in(RotationsPerSecond));
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
		if (getCurrentState() != null) {
			switch (getCurrentState()) {
				case IDLE_STATE:
					if (input != null && input.getButtonPressed(ButtonInput.PASSER_PREP_TOGGLE)) {
						pastState = getCurrentState();
						return ShooterFSMState.PASSER_PREP_STATE;
					} else if (input != null
						&& input.getButtonPressed(ButtonInput.SHOOTER_PREP_TOGGLE)) {
						pastState = getCurrentState();
						return ShooterFSMState.SHOOTER_PREP_STATE;
					} else if (input != null
						&& input.getButtonPressed(ButtonInput.MANUAL_SHOOT_TOGGLE)) {
						pastState = getCurrentState();
						return ShooterFSMState.MANUAL_PREP_STATE;
					} else {
						return getCurrentState();
					}

				case PASSER_PREP_STATE:
					if (input != null && input.getButtonValue(ButtonInput.IDLE_SHOOTER_TOGGLE)) {
						pastState = getCurrentState();
						return ShooterFSMState.IDLE_STATE;
					} else if (input != null
						&& input.getButtonPressed(ButtonInput.SHOOTER_PREP_TOGGLE)) {
						stopFlywheel();
						pastState = getCurrentState();
						return ShooterFSMState.SHOOTER_PREP_STATE;
					} else if (input != null
						&& input.getButtonPressed(ButtonInput.MANUAL_SHOOT_TOGGLE)) {
						stopFlywheel();
						pastState = getCurrentState();
						return ShooterFSMState.MANUAL_PREP_STATE;
					} else if (input != null
						&& isAtSpeed() && flywheelTargetSpeed.in(RotationsPerSecond) != 0
						&& input.getButtonValue(ButtonInput.REV_FEEDER)) {
						//need to change colors for if its at speed and at angle so that
						// they know when to pull triggers
						pastState = getCurrentState();
						resetStorage();
						return ShooterFSMState.FEED_STATE;
					} else {
						return getCurrentState();
					}

					case SHOOTER_PREP_STATE:
					if (input != null && input.getButtonPressed(ButtonInput.IDLE_SHOOTER_TOGGLE)) {
						pastState = getCurrentState();
						return ShooterFSMState.IDLE_STATE;
					} else if (input != null
						&& input.getButtonPressed(ButtonInput.PASSER_PREP_TOGGLE)) {
						stopFlywheel();
						pastState = getCurrentState();
						return ShooterFSMState.PASSER_PREP_STATE;
					} else if (input != null
						&& input.getButtonPressed(ButtonInput.MANUAL_SHOOT_TOGGLE)) {
						stopFlywheel();
						pastState = getCurrentState();
						return ShooterFSMState.MANUAL_PREP_STATE;
					} else if (input != null && isAtSpeed()
						&& flywheelTargetSpeed.in(RotationsPerSecond) != 0
						&& input.getButtonValue(ButtonInput.REV_FEEDER)) {
						pastState = getCurrentState();
						resetStorage();
						return ShooterFSMState.FEED_STATE;
					} else {
						return getCurrentState();
					}
					
				case FEED_STATE:
					if (!isAtSpeed() || input == null
						|| !input.getButtonValue(ButtonInput.REV_FEEDER)) {
						return pastState;
						//pastState should only store shooter_prep, passer_prep, and manual_prep
					} else if (input != null
						&& input.getButtonPressed(ButtonInput.IDLE_SHOOTER_TOGGLE)) {
						pastState = getCurrentState();
						return ShooterFSMState.IDLE_STATE;
					} else if (input != null
						&& input.getButtonPressed(ButtonInput.PASSER_PREP_TOGGLE)) {
						pastState = getCurrentState();
						return ShooterFSMState.PASSER_PREP_STATE;
					} else if (input != null
						&& input.getButtonPressed(ButtonInput.SHOOTER_PREP_TOGGLE)) {
						pastState = getCurrentState();
						return ShooterFSMState.SHOOTER_PREP_STATE;
					} else if (input != null
						&& input.getButtonPressed(ButtonInput.MANUAL_SHOOT_TOGGLE)) {
						pastState = getCurrentState();
						return ShooterFSMState.MANUAL_PREP_STATE;
					} else {
						return getCurrentState();
					}

				

				case MANUAL_PREP_STATE:
					// Manual can only go to idle (we need the button inputs for right and left
					// bumper to adjust manually)
					if (input != null && input.getButtonPressed(ButtonInput.IDLE_SHOOTER_TOGGLE)) {
						pastState = getCurrentState();
						return ShooterFSMState.IDLE_STATE;
					} else if (input != null //&& isAtSpeed()
						&& flywheelTargetSpeed.in(RotationsPerSecond) != 0
						&& input.getButtonValue(ButtonInput.REV_FEEDER)) {
						pastState = getCurrentState();
						//resetStorage();
						return ShooterFSMState.FEED_STATE;
					} else {
						return getCurrentState();
					}

				default:
					throw new IllegalStateException("Invalid state: "
					+ getCurrentState().toString());
			}
		}
		throw new IllegalStateException("No current state");
	}

	private void resetStorage() {
		feedTimer.reset();
		feedTimer.stop();
		noFuelStored = false;
		//spindexTimer.reset();
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in IDLE_STATE.
	 * @param input Global Input if robot in teleop mode or null if
	 *		the robot is in autonomous mode.
	 */
	private void handleIdleState(Input input) {
		flywheelTargetSpeed = RotationsPerSecond.of(0);
		//updateFlywheel();
		flywheelMotor.stopMotor();
		//updateHood();
		feederMotor.set(0);
		feedTimer.stop();
		feedTimer.reset();
		//set hoodMotor to 20 degrees/base angle?
		//hood remains at current angle.
	}
	/**
	 * Handle behavior in PASSER_PREP_STATE.
	 * @param input Global Input if robot in teleop mode or null if
	 *		the robot is in autonomous mode.
	 */
	private void handlePasserPrepState(Input input) {
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
		//feeder 0 is flywheel velocity, feeder 1 is hood angle
		double flyspeed = calculateTargetPassSpeed(correctTarget);
		flywheelTargetSpeed = RotationsPerSecond.of((double) flyspeed);

		updateFlywheel();
		// TODO: code to find the distance vector from where we are to passing targets
		// (preferably outpost and thelocation of outpost on the other side) (3d vector)
	}
	/**
	 * Calculate needed values to pass  in specific targets.
	 * @param target The pose we are targetting towards
	 * @return A double holding the target's needed flywheel speed
	 */
	public double calculateTargetPassSpeed(Pose2d target) {
		Transform2d transform = curPose.minus(target);
		double distance = Math.sqrt(Math.pow(transform.getX(), 2) + Math.pow(transform.getY(), 2));
		double flyspeed = ShooterConstants.PASSING_REGRESSION_CONSTANT
			+ ShooterConstants.PASSING_REGRESSION_SLOPE * Units.metersToInches(distance);
		return flyspeed;
		//code to be determined based off of regression model
	}

	/**
	 * Calculate needed values to shoot in specific targets.
	 * @param target The pose we are targetting towards
	 * @return A double holding the target's needed flywheel speed
	 */
	public double calculateTargetShootSpeed(Pose2d target) {
		return ShooterConstants.TEMP_FLYSPEED;
		//code to be determined based off of regression model
	}



	private void stopFlywheel() {
		flywheelTargetSpeed = RotationsPerSecond.of(0);
		updateFlywheel();
	}

	/**
	 * Check if we are currently in shooter prep state, which is used for the drivetrain.
	 * @return true if the past state is SHOOTER_PREP_STATE, false otherwise
	 */
	public boolean getIsPastStateShooterPrep() {
		return pastState == ShooterFSMState.SHOOTER_PREP_STATE;
	}

	/**
	 * Check if we are currently in shooter prep state, which is used for the drivetrain
	 * to know whether to face the hub or the pass zone.
	 * @return true if the current state is SHOOTER_PREP_STATE, false otherwise
	 */
	public boolean getIsCurrentStateShooterPrep() {
		return getCurrentState() == ShooterFSMState.SHOOTER_PREP_STATE;
	}


	/**
	 * Handle behavior in SHOOTER_PREP_STATE.
	 * @param input Global Input if robot in teleop mode or null if
	 *		the robot is in autonomous mode.
	 */
	private void handleShooterPrepState(Input input) {
		double flyspeed = calculateTargetShootSpeed(hubPose);
		flywheelTargetSpeed = RotationsPerSecond.of((double) flyspeed);

		updateFlywheel();
		//TBD: code to find the distance vector from where we are to hub center (3d vector)
	}

	/**
	 * Handle behavior in FEED_STATE.
	 * @param input Global Input if robot in teleop mode or null if
	 *		the robot is in autonomous mode.
	 */
	private void handleFeedState(Input input) {

		Logger.recordOutput("noFuelStored", noFuelStored);
		Logger.recordOutput("isIntakeDown", modelIntake(input));
		// if (!noFuelStored && !intake.isIntakeDownRunning()) {
		// //if (!noFuelStored && !modelIntake(input)) {
		// 	if (!isAtSpeed() || !input.getButtonValue(ButtonInput.REV_FEEDER)) {
		// 		feederMotor.stopMotor();
		// 		//spindexMotor.stopMotor();
		// 		//pastState should only store shooter_prep, passer_prep, and manual_prep
		// 	} else {
		// 		feederMotor.setControl(feederRequest.withVelocity(flywheelTargetSpeed.magnitude()));
		// 		//spindexMotor.setVoltage(spindexVoltage);
		// 	}

		// 	if (feedTimer.isRunning()) {
		// 		if (!breakBeam.get()) {
		// 			feedTimer.restart();
		// 		} else {
		// 			if (feedTimer.get() >= ShooterConstants.FEED_MAX_TIME) {
		// 				noFuelStored = true; //we don't have fuel
		// 			}
		// 		}

		// 	} else {
		// 		feedTimer.start();
		// 	}


		// } else if (intake.isIntakeDownRunning()) {
		// //} else if (modelIntake(input)) {
		// 	feedTimer.reset();
		// 	noFuelStored = false;
		// 	if (!isAtSpeed() || !input.getButtonValue(ButtonInput.REV_FEEDER)) {
		// 		feederMotor.stopMotor();
		// 		//spindexMotor.stopMotor();
		// 		//pastState should only store shooter_prep, passer_prep, and manual_prep
		// 	} else {
		// 		feederMotor.setControl(feederRequest.withVelocity(flywheelTargetSpeed.magnitude()));
		// 		//spindexMotor.setVoltage(spindexVoltage);
		// 	}
		// } else {
		// 	//condition for not having anyting stored
		// 	feederMotor.stopMotor();
		// 	//spindexMotor.stopMotor();
		// }


		//Switch spindexer back and forth
		// if (spindexTimer.advanceIfElapsed(ShooterConstants.SPINDEX_MAX_TIME)) {
		// 	spindexVoltage *= -1;
		// }
		
		if (!isAtSpeed() || !input.getButtonValue(ButtonInput.REV_FEEDER)) {
		//if (!input.getButtonValue(ButtonInput.REV_FEEDER)) {
			System.out.println("reach 2");
			feederMotor.stopMotor();
				//spindexMotor.stopMotor();
				//pastState should only store shooter_prep, passer_prep, and manual_prep
		} else {
			System.out.println("reach 1");
			feederMotor.setControl(feederRequest.withVelocity(ShooterConstants.
				FEEDER_CONSTANT_SPEED));
				//spindexMotor.setVoltage(ShooterConstants.SPINDEX_CONSTANT_VOLTAGE);
		}
		Logger.recordOutput("BreakBeam Timer: ", feedTimer.get());
	}

	/**
	 * Handle behavior in MANUAL_PREP_STATE.
	 * @param input Global Input if robot in teleop mode or null if
	 *		the robot is in autonomous mode.
	 */
	private void handleManualPrepState(Input input) {

		// FOR MANUAL ONLY: Right Bumper will be used as a deincrementer. Do not confuse this
		// with triggering Passer Prep.
		// FOR MANUAL ONLY: Left Bumper will be used to adjust hood angle. Do not confuse this
		// with triggering Shooter Prep.
		//how much the hood angle increases/decreases each click

		//shooter_prep_toggle will be for set manual points we can go to in comp
		// passer_prep_toggle will be the deincrementer
		// manual_shoot_toggle will be the flywheel control

		// for checkstyles
		//if hood is changing
		// boolean hoodSet = input.getButtonPressed(ButtonInput.SHOOTER_PREP_TOGGLE);

		//Deincrementing:
		boolean incrementSet = input.getButtonValue(ButtonInput.PASSER_PREP_TOGGLE);
		//Flywheel Speed Change:
		boolean flywheelSet = input.getButtonPressed(ButtonInput.MANUAL_SHOOT_TOGGLE);

		//Code for variable hood:
		// if (hoodSet && incrementSet) {
		// 	double hoodChangeMinus = hoodTargetAngle.in(Degrees) - hoodIncrement;
		// 	if (hoodChangeMinus >= ShooterConstants.HOOD_MIN_ANGLE.in(Degrees)) {
		// 		hoodTargetAngle = Degrees.of(hoodTargetAngle.in(Degrees) - hoodIncrement);
		// 	} else {
		// 		hoodTargetAngle = ShooterConstants.HOOD_MIN_ANGLE;
		// 	}
		// 	//decrease hood angle by 5 degrees
		// } else if (hoodSet) {
		// 	double hoodChangePlus = hoodTargetAngle.in(Degrees) + hoodIncrement;
		// 	if (hoodChangePlus <= ShooterConstants.HOOD_MAX_ANGLE.in(Degrees)) {
		// 		hoodTargetAngle = Degrees.of(hoodTargetAngle.in(Degrees) + hoodIncrement);
		// 	} else {
		// 		hoodTargetAngle = ShooterConstants.HOOD_MAX_ANGLE;
		// 	}
		// 	//increase hood angle by 5 degrees
		// }
		// updateHood();

		double flyIncrement = ShooterConstants.FLYWHEEL_INCREMENTER.in(RotationsPerSecond);
		//how much the flywheel speed increases/decreases each click
		if (flywheelSet && incrementSet) {
			var change = flywheelTargetSpeed.in(RotationsPerSecond) - flyIncrement;
			if (change > 0) {
				flywheelTargetSpeed = RotationsPerSecond.of(change);
				flywheelMotorStopped = false;
			} else {
				flywheelTargetSpeed = RotationsPerSecond.of(0);
				flywheelMotorStopped = true;
			}
			//decrease flywheel speed by some constant, right now set to 10 m/s
		} else if (flywheelSet) {
			var change = flywheelTargetSpeed.in(RotationsPerSecond) + flyIncrement;
			if (change < ShooterConstants.FLYWHEEL_MAX_SPEED.in(RotationsPerSecond)) {
				flywheelTargetSpeed = RotationsPerSecond.of(change);
				flywheelMotorStopped = false;

			} else {
				flywheelTargetSpeed = ShooterConstants.FLYWHEEL_MAX_SPEED;
				flywheelMotorStopped = false;
			}

			//increase flywheel speed by some constant, right now set to 10 m/s
		}

		if (!flywheelMotorStopped) {
			updateFlywheel();
		} else {
			flywheelMotor.stopMotor();
		}


		// check if current speed of motors and current angle matches what we just set it to there
		// with the boolean conditions
	}

	private void updateFlywheel() {
		Logger.recordOutput("Flywheel Target Speed", flywheelTargetSpeed);
		flywheelMotor.setControl(flywheelRequest.withVelocity(
			flywheelTargetSpeed.in(RotationsPerSecond)));

	}


	private boolean modelIntake(Input input) {
		if (input.getButtonValue(ButtonInput.SHOOTER_PREP_TOGGLE)) {
			return true;
		}
		return false;
	}



	// private void updateHood() {
	// 	hoodMotor.setControl(hoodRequest.withPosition(hoodTargetAngle.magnitude()));
	// 	hoodAngle = Degrees.of(hoodMotor.getPosition().getValue().in(Units.Degrees));
	// }
}
