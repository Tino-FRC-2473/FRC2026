package frc.robot.systems;

import edu.wpi.first.wpilibj.DriverStation;

import org.littletonrobotics.junction.Logger;
// Third party Hardware Imports
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

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
	private TalonFX flywheelMotor;
	private TalonFX feederMotor;
	private SparkMax spindexMotor;
	private Measure<AngularVelocityUnit> flywheelSpeed; //Units.RotationsPerSecond
	private Measure<AngularVelocityUnit> flywheelTargetSpeed; //Units.RotationsPerSecond
	private Measure<AngleUnit> hoodAngle; //Units.Degrees
	private ShooterFSMState pastState;
	private TalonFXConfiguration flywheelConfigs;
	private TalonFXConfiguration feederConfigs;
	private Drivetrain drivetrain;
	private MotionMagicVelocityVoltage flywheelRequest;
	private MotionMagicVelocityVoltage feederRequest;
	private IntakeFSM intake;
	private DigitalInput breakBeam;
	private Timer feedTimer = new Timer();
	private boolean noFuelStored = false;
	private double hubDistance;
	private double outpostDistance;
	private double target3Distance;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public ShooterFSMSystem() {
		curPose = new Pose2d();
		// flywheelRequest = new MotionMagicVelocityVoltage(0);
		// feederRequest = new MotionMagicVelocityVoltage(0);
		// flywheelMotor = new TalonFXWrapper(
		// 	HardwareMap.CAN_ID_FLYWHEEL
		// );
		// feederMotor = new TalonFXWrapper(
		// 	HardwareMap.CAN_ID_FEEDER
		// );
		// spindexMotor = new SparkMax(HardwareMap.CAN_ID_SPINDEXER,
		// 	MotorType.kBrushless);


		// spindexMotor = new TalonFXWrapper(HardwareMap.CAN_ID_SPINDEXER);
		// spindexConfigs = new TalonFXConfiguration();

		// var spindexFeedbackConfigs = spindexConfigs.Feedback;
		// var spindexRatio =
		// 	ShooterConstants.SPINDEX_GEAR_RATIO;
		// spindexFeedbackConfigs.SensorToMechanismRatio = spindexRatio;
		// //set to 2 (divided by 360 to get in terms of degrees)

		// spindexMotor.getConfigurator().apply(spindexConfigs);
		// var limitConfigs = new CurrentLimitsConfigs();

		// // enable stator current limit
		// limitConfigs.StatorCurrentLimit = ShooterConstants.SHOOTER_CURRENT_LIMIT;
		// limitConfigs.StatorCurrentLimitEnable = true;

		// flywheelConfigs = new TalonFXConfiguration();
		// var flywheel0Config = flywheelConfigs.Slot0;
		// //voltage output to overcome static friction
		// flywheel0Config.kS = ShooterConstants.FLYWHEEL_MM_CONSTANT_S;
		// //voltage for 1 rps in the motor, 0.11
		// flywheel0Config.kV = ShooterConstants.MM_CONSTANT_V;
		// //account for position error of 1 rotation
		// flywheel0Config.kP = ShooterConstants.FLYWHEEL_MM_CONSTANT_P;
		// //output for integrated error
		// flywheel0Config.kI = ShooterConstants.FLYWHEEL_MM_CONSTANT_I;
		// //account for velocity error of 1rps
		// flywheel0Config.kD = ShooterConstants.FLYWHEEL_MM_CONSTANT_D;

		outpostPose = ShooterConstants.OUTPOST_POSE;
		hubPose = ShooterConstants.HUB_POSE;
		target3Pose = ShooterConstants.TARGET3_POSE;
		hoodAngle = ShooterConstants.HOOD_ANGLE;

		// var flywheelMotionMagicConfigs = flywheelConfigs.MotionMagic;
		// //160 rps/s
		// flywheelMotionMagicConfigs.MotionMagicAcceleration =
		// 	ShooterConstants.MAGIC_ACCELERATION.in(RotationsPerSecondPerSecond);
		// //1600 rps/s/s, 10* acceleration
		// flywheelMotionMagicConfigs.MotionMagicJerk = ShooterConstants.MAGIC_JERK;

		// var flywheelFeedbackConfigs = flywheelConfigs.Feedback;
		// //set to 3
		// flywheelFeedbackConfigs.SensorToMechanismRatio = ShooterConstants.FLYWHEEL_GEAR_RATIO;

		// flywheelMotor.getConfigurator().apply(flywheelConfigs);
		// flywheelMotor.getConfigurator().apply(limitConfigs);

		// feederConfigs = new TalonFXConfiguration();
		// var feeder0Config = feederConfigs.Slot0;
		// feeder0Config.kS = ShooterConstants.FEEDER_MM_CONSTANT_S;
		// feeder0Config.kV = ShooterConstants.MM_CONSTANT_V;
		// feeder0Config.kP = ShooterConstants.FEEDER_MM_CONSTANT_P;
		// feeder0Config.kI = ShooterConstants.FEEDER_MM_CONSTANT_I;
		// feeder0Config.kD = ShooterConstants.FEEDER_MM_CONSTANT_D;

		// var feederMotionMagicConfigs = feederConfigs.MotionMagic;
		// feederMotionMagicConfigs.MotionMagicAcceleration =
		// 	ShooterConstants.MAGIC_ACCELERATION.in(RotationsPerSecondPerSecond);
		// feederMotionMagicConfigs.MotionMagicJerk = ShooterConstants.MAGIC_JERK;
		// var feederFeedbackConfigs = feederConfigs.Feedback;
		// //set to 3
		// feederFeedbackConfigs.SensorToMechanismRatio = ShooterConstants.FEEDER_GEAR_RATIO;

		// feederMotor.getConfigurator().apply(feederConfigs);
		// feederMotor.getConfigurator().apply(limitConfigs);

		// // BaseStatusSignal.setUpdateFrequencyForAll(
		// // 		ShooterConstants.UPDATE_FREQUENCY_HZ,
		// // 		spindexMotor.getPosition(),
		// // 		spindexMotor.getVelocity(),
		// // 		spindexMotor.getAcceleration(),
		// // 		spindexMotor.getMotorVoltage(),
		// // 		spindexMotor.getRotorPosition(),
		// // 		spindexMotor.getRotorVelocity()
		// // );

		// BaseStatusSignal.setUpdateFrequencyForAll(
		// 		ShooterConstants.UPDATE_FREQUENCY_HZ,
		// 		feederMotor.getPosition(),
		// 		feederMotor.getVelocity(),
		// 		feederMotor.getAcceleration(),
		// 		feederMotor.getMotorVoltage(),
		// 		feederMotor.getRotorPosition(),
		// 		feederMotor.getRotorVelocity()
		// );

		// BaseStatusSignal.setUpdateFrequencyForAll(
		// 		ShooterConstants.UPDATE_FREQUENCY_HZ,
		// 		flywheelMotor.getPosition(),
		// 		flywheelMotor.getVelocity(),
		// 		flywheelMotor.getAcceleration(),
		// 		flywheelMotor.getMotorVoltage(),
		// 		flywheelMotor.getRotorPosition(),
		// 		flywheelMotor.getRotorVelocity()
		// );

		// // spindexMotor.optimizeBusUtilization();
		// feederMotor.optimizeBusUtilization();
		// flywheelMotor.optimizeBusUtilization();

		// breakBeam = new DigitalInput(HardwareMap.STORAGE_BREAK_BEAM_DIO_PORT);
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
	public ShooterFSMSystem(Drivetrain driveSystem, IntakeFSM intakeSystem) {
		// Perform hardware init using a wrapper class
		// this is so we can see motor outputs during simulatiuons
		this();
		// drivetrain = driveSystem;
		// curPose = drivetrain.getPose();
		this.intake = intakeSystem;
		this.drivetrain = driveSystem;
	}

	/* ======================== Public methods ======================== */

	// overridden methods don't require javadocs
	// however, you may want to add implementation specific javadocs
	/**
	 * Checks if the target flywheel speed matches the actual flywheel speed within margin of error.
	 * @return Boolean statement whether or not it is at flywheel speed or not
	 */
	public boolean isAtSpeed() {
		Logger.recordOutput("Actual Motor Speed", flywheelSpeed.in(RotationsPerSecond));
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
		setCurrentState(ShooterFSMState.SHOOTER_PREP_STATE);

		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}

	@Override
	public void update(Input input) {
		if (drivetrain != null) {
			curPose = drivetrain.getPose();
		}
		//curPose = drivetrain.getPose();
		if (getCurrentState() != null) {
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

				case FEED_STATE:
					handleFeedState((TeleopInput) input);
					break;

				case MANUAL_PREP_STATE:
					handleManualPrepState((TeleopInput) input);
					break;

				default:
					throw new IllegalStateException("Invalid state: "
					+ getCurrentState().toString());
			}
		}
		//System.out.println("Update Cur State" + getCurrentState());
		setCurrentState(nextState(input));
	}


	/**
	 * Finds the field pose that the robot should align with based on its current state.
	 * @return The pose of the target if it is in preperation state,
	 * null if the bot is in either Idle, Feed, or Manual states
	 */
	// Fix the distance calculation logic here
	public Pose2d getAutoTargetPose() {
		System.out.println("1");
		ShooterFSMState state = getCurrentState();
		if (state != ShooterFSMState.SHOOTER_PREP_STATE
			&& state != ShooterFSMState.PASSER_PREP_STATE) {
			System.out.println("2");
			return null;
		}

		// Safety check: ensure we have odometry data
		if (curPose == null) {
			return null;
		}

		var alliance = DriverStation.getAlliance();
		boolean isRed = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;

		Pose2d activeHub = isRed ? mirrorPose(hubPose) : hubPose;
		Pose2d activeOutpost = isRed ? mirrorPose(outpostPose) : outpostPose;
		Pose2d activeTarget3 = isRed ? mirrorPose(target3Pose) : target3Pose;

		hubDistance = curPose.getTranslation().getDistance(activeHub.getTranslation());
		outpostDistance = curPose.getTranslation().getDistance(activeOutpost.getTranslation());
		target3Distance = curPose.getTranslation().getDistance(activeTarget3.getTranslation());

		if (getCurrentState() == ShooterFSMState.SHOOTER_PREP_STATE) {
			return activeHub;
		} else {
			// Now distance check is accurate for both alliances
			return (outpostDistance < target3Distance) ? activeOutpost : activeTarget3;
		}
	}

	private Pose2d mirrorPose(Pose2d pose) {
		double fieldLength = ShooterConstants.FIELD_LENGTH;
		return new Pose2d(
			fieldLength - pose.getX(),
			pose.getY(),
			pose.getRotation().plus(Rotation2d.fromDegrees(180))
		);
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
		return getCurrentState();
		// if (getCurrentState() != null) {
		// 	switch (getCurrentState()) {
		// 		case IDLE_STATE:
		// 			if (input != null && input.getButtonPressed(ButtonInput.PASSER_PREP_TOGGLE)) {
		// 				pastState = getCurrentState();
		// 				return ShooterFSMState.PASSER_PREP_STATE;
		// 			} else if (input != null
		// 				&& input.getButtonPressed(ButtonInput.SHOOTER_PREP_TOGGLE)) {
		// 				pastState = getCurrentState();
		// 				return ShooterFSMState.SHOOTER_PREP_STATE;
		// 			} else if (input != null
		// 				&& input.getButtonPressed(ButtonInput.MANUAL_SHOOT_TOGGLE)) {
		// 				pastState = getCurrentState();
		// 				return ShooterFSMState.MANUAL_PREP_STATE;
		// 			} else {
		// 				return getCurrentState();
		// 			}

		// 		case PASSER_PREP_STATE:
		// 			if (input != null && input.getButtonPressed(ButtonInput.IDLE_SHOOTER_TOGGLE)) {
		// 				pastState = getCurrentState();
		// 				return ShooterFSMState.IDLE_STATE;
		// 			} else if (input != null
		// 				&& input.getButtonPressed(ButtonInput.SHOOTER_PREP_TOGGLE)) {
		// 				stopFlywheel();
		// 				pastState = getCurrentState();
		// 				return ShooterFSMState.SHOOTER_PREP_STATE;
		// 			} else if (input != null
		// 				&& input.getButtonPressed(ButtonInput.MANUAL_SHOOT_TOGGLE)) {
		// 				stopFlywheel();
		// 				pastState = getCurrentState();
		// 				return ShooterFSMState.MANUAL_PREP_STATE;
		// 			} else if (input != null
		// 				&& isAtSpeed() && flywheelTargetSpeed.in(RotationsPerSecond) != 0
		// 				&& input.getButtonValue(ButtonInput.REV_FEEDER)) {
		// 				//need to change colors for if its at speed and at angle so that
		// 				// they know when to pull triggers
		// 				pastState = getCurrentState();
		// 				return ShooterFSMState.FEED_STATE;
		// 			} else {
		// 				return getCurrentState();
		// 			}

		// 		case FEED_STATE:
		// 			if (!isAtSpeed() || input == null
		// 				|| !input.getButtonValue(ButtonInput.REV_FEEDER)) {
		// 				return pastState;
		// 				//pastState should only store shooter_prep, passer_prep, and manual_prep
		// 			} else if (input != null
		// 				&& input.getButtonPressed(ButtonInput.IDLE_SHOOTER_TOGGLE)) {
		// 				pastState = getCurrentState();
		// 				return ShooterFSMState.IDLE_STATE;
		// 			} else if (input != null
		// 				&& input.getButtonPressed(ButtonInput.PASSER_PREP_TOGGLE)) {
		// 				pastState = getCurrentState();
		// 				return ShooterFSMState.PASSER_PREP_STATE;
		// 			} else if (input != null
		// 				&& input.getButtonPressed(ButtonInput.SHOOTER_PREP_TOGGLE)) {
		// 				pastState = getCurrentState();
		// 				return ShooterFSMState.SHOOTER_PREP_STATE;
		// 			} else if (input != null
		// 				&& input.getButtonPressed(ButtonInput.MANUAL_SHOOT_TOGGLE)) {
		// 				pastState = getCurrentState();
		// 				return ShooterFSMState.MANUAL_PREP_STATE;
		// 			} else {
		// 				return getCurrentState();
		// 			}

		// 		case SHOOTER_PREP_STATE:
		// 			if (input != null && input.getButtonPressed(ButtonInput.IDLE_SHOOTER_TOGGLE)) {
		// 				pastState = getCurrentState();
		// 				return ShooterFSMState.IDLE_STATE;
		// 			} else if (input != null
		// 				&& input.getButtonPressed(ButtonInput.PASSER_PREP_TOGGLE)) {
		// 				stopFlywheel();
		// 				pastState = getCurrentState();
		// 				return ShooterFSMState.PASSER_PREP_STATE;
		// 			} else if (input != null
		// 				&& input.getButtonPressed(ButtonInput.MANUAL_SHOOT_TOGGLE)) {
		// 				stopFlywheel();
		// 				pastState = getCurrentState();
		// 				return ShooterFSMState.MANUAL_PREP_STATE;
		// 			} else if (input != null && isAtSpeed()
		// 				&& flywheelTargetSpeed.in(RotationsPerSecond) != 0
		// 				&& input.getButtonValue(ButtonInput.REV_FEEDER)) {
		// 				pastState = getCurrentState();
		// 				return ShooterFSMState.FEED_STATE;
		// 			} else {
		// 				return getCurrentState();
		// 			}

		// 		case MANUAL_PREP_STATE:
		// 			// Manual can only go to idle (we need the button inputs for right and left
		// 			// bumper to adjust manually)
		// 			if (input != null && input.getButtonPressed(ButtonInput.IDLE_SHOOTER_TOGGLE)) {
		// 				pastState = getCurrentState();
		// 				return ShooterFSMState.IDLE_STATE;
		// 			} else if (input != null && isAtSpeed()
		// 				&& flywheelTargetSpeed.in(RotationsPerSecond) != 0
		// 				&& input.getButtonValue(ButtonInput.REV_FEEDER)) {
		// 				pastState = getCurrentState();
		// 				return ShooterFSMState.FEED_STATE;
		// 			} else {
		// 				return getCurrentState();
		// 			}

		// 		default:
		// 			throw new IllegalStateException("Invalid state: "
		// 			+ getCurrentState().toString());
		// 	}
		// }
		// throw new IllegalStateException("No current state");
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in IDLE_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *		the robot is in autonomous mode.
	 */
	private void handleIdleState(TeleopInput input) {
		// flywheelTargetSpeed = RotationsPerSecond.of(0);
		// updateFlywheel();
		//updateHood();
		// feederMotor.set(0);
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

		outpostDistance = (double) curPose.getTranslation()
				.getDistance(outpostPose.getTranslation());
		target3Distance = (double) curPose.getTranslation()
				.getDistance(target3Pose.getTranslation());
		if (outpostDistance < target3Distance) {
			correctTarget = target3Pose;
		} else {
			correctTarget = outpostPose;
		}

		//feeder 0 is flywheel velocity, feeder 1 is hood angle
		double flyspeed = calculateTargetFlyspeed(correctTarget);
		flywheelTargetSpeed = RotationsPerSecond.of((double) flyspeed);

		updateFlywheel();
		//updateHood();
		// TBD: code to find the distance vector from where we are to passing targets
		// (preferably outpost and thelocation of outpost on the other side) (3d vector)
	}
	/**
	 * Calculate needed values to shoot in specific targets.
	 * @param target The pose we are targetting towards
	 * @return A list holding the target's needed flywheel speed and hood angle in that
	 * order to make a pass
	 */
	public double calculateTargetFlyspeed(Pose2d target) {
		return ShooterConstants.TEMP_FLYSPEED;
		//code to be determined based off of regression model
	}

	private void stopFlywheel() {
		flywheelTargetSpeed = RotationsPerSecond.of(0);
		updateFlywheel();
	}

	/**
	 * Handle behavior in SHOOTER_PREP_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *		the robot is in autonomous mode.
	 */
	private void handleShooterPrepState(TeleopInput input) {
		// double flyspeed = calculateTargetFlyspeed(hubPose);
		// flywheelTargetSpeed = RotationsPerSecond.of((double) flyspeed);
		//drivetrain.targetHub();

		// updateFlywheel();
		//TBD: code to find the distance vector from where we are to hub center (3d vector)
	}

	/**
	 * Handle behavior in FEED_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *		the robot is in autonomous mode.
	 */
	private void handleFeedState(TeleopInput input) {
		if (!noFuelStored && !intake.isIntakeDownRunning()) {
			if (!isAtSpeed() || !input.getButtonValue(ButtonInput.REV_FEEDER)) {
				feederMotor.stopMotor();
				spindexMotor.stopMotor();
				//pastState should only store shooter_prep, passer_prep, and manual_prep
			} else {
				feederMotor.setControl(feederRequest.withVelocity(flywheelTargetSpeed.magnitude()));
				spindexMotor.setVoltage(ShooterConstants.SPINDEX_CONSTANT_VOLTAGE);
			}


			if (feedTimer.isRunning()) {
				if (!breakBeam.get()) {
					feedTimer.restart();
				} else {
					if (feedTimer.get() >= ShooterConstants.FEED_MAX_TIME) {
						noFuelStored = true; //we don't have fuel
					}
				}

			} else {
				feedTimer.start();
			}
		} else if (intake.isIntakeDownRunning()) {
			noFuelStored = false;
			if (!isAtSpeed() || !input.getButtonValue(ButtonInput.REV_FEEDER)) {
				feederMotor.stopMotor();
				spindexMotor.stopMotor();
				//pastState should only store shooter_prep, passer_prep, and manual_prep
			} else {
				feederMotor.setControl(feederRequest.withVelocity(flywheelTargetSpeed.magnitude()));
				spindexMotor.setVoltage(ShooterConstants.SPINDEX_CONSTANT_VOLTAGE);
			}
		} else {
			//condition for not having anyting stored
			feederMotor.stopMotor();
			spindexMotor.stopMotor();
		}
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
		// double hoodIncrement = ShooterConstants.HOOD_INCREMENTER.in(Degrees);
		// for checkstyles
		//if hood is changing
		// boolean hoodSet = input.getButtonPressed(ButtonInput.SHOOTER_PREP_TOGGLE);
		//if we are deincrementing
		boolean incrementSet = input.getButtonValue(ButtonInput.PASSER_PREP_TOGGLE);
		//if flywheel is changing
		boolean flywheelSet = input.getButtonPressed(ButtonInput.MANUAL_SHOOT_TOGGLE);

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
			} else {
				flywheelTargetSpeed = RotationsPerSecond.of(0);
			}
			//decrease flywheel speed by some constant, right now set to 10 m/s
		} else if (flywheelSet) {
			var change = flywheelTargetSpeed.in(RotationsPerSecond) + flyIncrement;
			if (change < ShooterConstants.FLYWHEEL_MAX_SPEED.in(RotationsPerSecond)) {
				flywheelTargetSpeed = RotationsPerSecond.of(change);
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
		Logger.recordOutput("Flywheel Target Speed", flywheelTargetSpeed);
		flywheelMotor.setControl(flywheelRequest.withVelocity(
			flywheelTargetSpeed.in(RotationsPerSecond)));
		double curSpeed = flywheelMotor.getVelocity().getValue().in(RotationsPerSecond);
		flywheelSpeed = RotationsPerSecond.of(curSpeed * ShooterConstants.FLYWHEEL_GEAR_RATIO);
	}

	// private void updateHood() {
	// 	hoodMotor.setControl(hoodRequest.withPosition(hoodTargetAngle.magnitude()));
	// 	hoodAngle = Degrees.of(hoodMotor.getPosition().getValue().in(Units.Degrees));
	// }
}
