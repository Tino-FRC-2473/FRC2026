package frc.robot.systems;

// WPILib Imports
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Units;

// Third party Hardware Imports
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;


import java.util.ArrayList;
import java.util.List;


// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.motors.SparkMaxWrapper;
import frc.robot.motors.TalonFXWrapper;
import frc.robot.HardwareMap;
import frc.robot.systems.AutoHandlerSystem.AutoFSMState;
import frc.robot.systems.Drivetrain;
import frc.robot.Constants.ShooterConstants;


enum FSMState {
	IDLE_STATE,
	SHOOTER_PREP_STATE,
	PASSER_PREP_STATE,
	INTAKE_STATE,
	MANUAL_PREP_STATE,
}

public class ShooterFSMSystem extends FSMSystem<FSMState> {
	//IMPORTANT NOTE: THIS WILL NOT BUILD AS THIS IS JUST A FRAMEWORK AND DOES NOT HAVE ANY DEFINED INPUT FUNCTIONS OR CONSTANT FIELDS YET
	/* ======================== Constants ======================== */

	private static final float MOTOR_RUN_POWER = 0.1f;
	

	/* ======================== Private variables ======================== */

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	//for curPose, we need to find the height of the shooter from the floor for calculations.
	private Pose2d curPose;
	private Pose2d outpostPose;
	private Pose2d hubPose;
	private Pose2d target3Pose; //probably going to be the mirrored side of the outpost
	private List<Pose2d> targetPoses;
	private TalonFX flywheelMotor;
	private TalonFX indexMotor;
	private TalonFX hoodMotor;
	private double flywheelSpeed;
	private double flywheelTargetSpeed;
	private double hoodAngle;
	private double hoodTargetAngle;
	private FSMState pastState;
	private TalonFXConfiguration hoodConfigs;
	private TalonFXConfiguration flywheelConfigs;
	private TalonFXConfiguration indexConfigs;
	private DriveFSMSystem drivetrain;
	
	

	
	

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public ShooterFSMSystem(){
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
		
		var hoodSoftwareLimitSwitchConfigs = hoodConfigs.SoftwareLimitSwitch;
		hoodSoftwareLimitSwitchConfigs.ForwardSoftLimitEnable = true;
		hoodSoftwareLimitSwitchConfigs.ForwardSoftLimitThreshold = 70;
		hoodSoftwareLimitSwitchConfigs.ReverseSoftLimitEnable = true;
		hoodSoftwareLimitSwitchConfigs.ReverseSoftLimitThreshold = 45;

		var hood0Config = hoodConfigs.Slot0;
		hood0Config.GravityType = GravityTypeValue.Arm_Cosine;
		hood0Config.kG = ShooterConstants.HOOD_MM_CONSTANT_G; //voltage output to overcome gravity
		hood0Config.kS = ShooterConstants.HOOD_MM_CONSTANT_S; //voltage output to overcome static friction
		hood0Config.kV = ShooterConstants.MM_CONSTANT_V; //voltage for 1 rps in the motor, 0.12+++
		hood0Config.kA = ShooterConstants.MM_CONSTANT_A; //voltage for acceleration for 1 rps in the motor
		hood0Config.kP = ShooterConstants.HOOD_MM_CONSTANT_P; //account for position error of 1 rotation
		hood0Config.kI = ShooterConstants.HOOD_MM_CONSTANT_I; //output for integrated error
		hood0Config.kD = ShooterConstants.HOOD_MM_CONSTANT_D; //account for velocity error of 1rps

		var hoodMotionMagicConfigs = hoodConfigs.MotionMagic;
		hoodMotionMagicConfigs.MotionMagicCruiseVelocity = ShooterConstants.HOOD_VELOCITY;
		hoodMotionMagicConfigs.MotionMagicAcceleration = ShooterConstants.HOOD_ACCELERATION;
		hoodMotionMagicConfigs.MotionMagicJerk = ShooterConstants.HOOD_JERK;

		var hoodFeedbackConfigs = hoodConfigs.Feedback;
		hoodFeedbackConfigs.SensorToMechanismRatio = ShooterConstants.HOOD_GEAR_RATIO / 360; //set to 2 (divided by 360 to get in terms of degrees)

		hoodMotor.getConfigurator().apply(hoodConfigs);

		flywheelConfigs = new TalonFXConfiguration();
		var flywheel0Config = flywheelConfigs.Slot0; 
		flywheel0Config.kS = ShooterConstants.FLYWHEEL_MM_CONSTANT_S; //voltage output to overcome static friction
		flywheel0Config.kV = ShooterConstants.MM_CONSTANT_V; //voltage for 1 rps in the motor, 0.12
		flywheel0Config.kA = ShooterConstants.MM_CONSTANT_A; //voltage for acceleration for 1 rps in the motor
		flywheel0Config.kP = ShooterConstants.FLYWHEEL_MM_CONSTANT_P; //account for position error of 1 rotation
		flywheel0Config.kI = ShooterConstants.FLYWHEEL_MM_CONSTANT_I; //output for integrated error
		flywheel0Config.kD = ShooterConstants.FLYWHEEL_MM_CONSTANT_D; //account for velocity error of 1rps

		var flywheelMotionMagicConfigs = flywheelConfigs.MotionMagic;
		flywheelMotionMagicConfigs.MotionMagicAcceleration = ShooterConstants.FLYWHEEL_ACCELERATION; //160 rps/s
		flywheelMotionMagicConfigs.MotionMagicJerk = ShooterConstants.FLYWHEEL_JERK; //1600 rps/s/s, 10* acceleration

		var flywheelFeedbackConfigs = flywheelConfigs.Feedback;
		flywheelFeedbackConfigs.SensorToMechanismRatio = ShooterConstants.FLYWHEEL_GEAR_RATIO; //set to 3

		flywheelMotor.getConfigurator().apply(flywheelConfigs);

		indexConfigs = new TalonFXConfiguration();
		var indexFeedbackConfigs = indexConfigs.Feedback;
		indexFeedbackConfigs.SensorToMechanismRatio = ShooterConstants.INDEXER_GEAR_RATIO; //set to 3

		indexMotor.getConfigurator().apply(indexConfigs);

		hoodMotor.setPosition(70);

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
	
	public ShooterFSMSystem(Drivetrain driveSystem) {
		// Perform hardware init using a wrapper class
		// this is so we can see motor outputs during simulatiuons
		ShooterFSMSystem();
		drivetrain = driveSystem;
		curPose = drivetrain.getPose();
	}

	/* ======================== Public methods ======================== */

	// overridden methods don't require javadocs
	// however, you may want to add implementation specific javadocs

	public boolean isAtSpeed(){
		return (Math.abs(flywheelTargetSpeed - flywheelSpeed) <= ShooterConstants.FLYWHEEL_MOE);
	}

	public boolean isAtAngle(){
		return (Math.abs(hoodTargetAngle - hoodAngle) < ShooterConstants.HOOD_MOE);
	}
	
	@Override
	public void reset() {
		setCurrentState(FSMState.IDLE_STATE);

		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}

	@Override
	public void update(TeleopInput input) {
		curPose = drivetrain.getPose();
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

			case INTAKE_STATE:
				handleIntakeState(input);
				break;

			case MANUAL_PREP_STATE:
				handleManualPrepState(input);
				break;
			
			default:
				throw new IllegalStateException("Invalid state: " + getCurrentState().toString());
		}
		setCurrentState(nextState(input));
	}

	@Override
	public boolean updateAutonomous(AutoFSMState autoState) {
		switch (autoState) {
			case STATE1:
				return handleAutoState1();
			case STATE2:
				return handleAutoState2();
			case STATE3:
				return handleAutoState3();
			default:
				return true;
		}
	}

	/* ======================== Protected methods ======================== */

	@Override
	protected FSMState nextState(TeleopInput input) {
		switch (getCurrentState()) {
			case IDLE_STATE:
				if (input != null && input.isRightBumperPressed()) {
					pastState = getCurrentState();
					return FSMState.PASSER_PREP_STATE;
				} else if (input != null && input.isLeftBumperPressed()) {
					pastState = getCurrentState();
					return FSMState.SHOOTER_PREP_STATE;
				} else if (input != null && input.isLeftTriggerPressed()){
					pastState = getCurrentState();
					return FSMState.MANUAL_PREP_STATE;
				}

			case PASSER_PREP_STATE:
				if (input.isTouchpadPressed()){
					pastState = getCurrentState();
					return FSMState.IDLE_STATE;
				}
				if (input.isLeftBumperPressed()){
					pastState = getCurrentState();
					return FSMState.SHOOTER_PREP_STATE;
				} 
				if (input.isLeftTriggerPressed()){
					pastState = getCurrentState();
					return FSMState.MANUAL_PREP_STATE;
				}
				if (isAtSpeed() && isAtAngle() && input.isRightTriggerPressed()){ //need to make sure to change colors for if its at speed and at angle so that they know when to pull triggers
					pastState = getCurrentState();
					return FSMState.INTAKE_STATE;
				}

			case INTAKE_STATE:
				if (!isAtSpeed() || !isAtAngle() || !input.isRightTriggerPressed()){
					indexMotor.set(0);
					return pastState; //pastState should only store shooter_prep, passer_prep, and manual_prep
				}

				if (input.isTouchpadPressed()){
					pastState = getCurrentState();
					return FSMState.IDLE_STATE;
				}

				if (input.isRightBumperPressed()){
					pastState = getCurrentState();
					return FSMState.PASSER_PREP_STATE;
				}

				if (input.isLeftBumperPressed()){
					pastState = getCurrentState();
					return FSMState.SHOOTER_PREP_STATE;
				}

				if (input.isLeftTriggerPressed()){
					pastState = getCurrentState();
					return FSMState.MANUAL_PREP_STATE;
				}
			
			case SHOOTER_PREP_STATE:
				if (input.isTouchpadPressed()){
					pastState = getCurrentState();
					return FSMState.IDLE_STATE;
				}
				if (input.isRightBumperPressed()){
					pastState = getCurrentState();
					return FSMState.PASSER_PREP_STATE;
				} 
				if (input.isLeftTriggerPressed()){
					pastState = getCurrentState();
					return FSMState.MANUAL_PREP_STATE;
				}
				if (isAtSpeed() && isAtAngle() && input.isRightTriggerPressed()){
					pastState = getCurrentState();
					return FSMState.INTAKE_STATE;
				}

			case MANUAL_PREP_STATE:
			//Manual can only go to idle (we need the button inputs for right and left bumper to adjust manually)
				if (input.isTouchpadPressed()){
					pastState = getCurrentState();
					return FSMState.IDLE_STATE;
				}

				if (isAtSpeed() && isAtAngle() && input.isRightTriggerPressed()){
					pastState = getCurrentState();
					return FSMState.INTAKE_STATE;
				}
				

			default:
				throw new IllegalStateException("Invalid state: " + getCurrentState().toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in IDLE_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleIdleState(TeleopInput input) {
		flywheelTargetSpeed = 0;
		hoodTargetAngle = 70;
		updateFlywheel();
		updateHood();
		indexMotor.set(0);
		//set hoodMotor to 20 degrees/base angle?
		//hood remains at current angle.

	}
	/**
	 * Handle behavior in PASSER_PREP_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handlePasserPrepState(TeleopInput input) {
		Pose2d correctTarget = new Pose2d();

		double outpostDistance = (double)curPose.getTranslation().getDistance(outpostPose.getTranslation());
		double target3Distance = (double)curPose.getTranslation().getDistance(target3Pose.getTranslation());
		if (outpostDistance < target3Distance){
			correctTarget = target3Pose;
		} else {
			correctTarget = outpostPose;
		}

		List<Object> targetValues = calculateTargetValues(correctTarget); //index 0 is flywheel velocity, index 1 is hood angle
		flywheelTargetSpeed = (double)targetValues.get(0);
		hoodTargetAngle = (double)targetValues.get(1);

		updateFlywheel();
		updateHood();
		//TBD: code to find the distance vector from where we are to passing targets (preferably outpost and the location of outpost on the other side) (3d vector)
		
	}

	public List<Object> calculateTargetValues(Pose2d target){
		return null;
		//code to be determined based off of regression model
	}

	/**
	 * Handle behavior in SHOOTER_PREP_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleShooterPrepState(TeleopInput input) {
		List<Object> targetValues = calculateTargetValues(hubPose);
		flywheelTargetSpeed = (double)targetValues.get(0);
		hoodTargetAngle = (double)targetValues.get(1);

		updateFlywheel();
		//TBD: code to find the distance vector from where we are to hub center (3d vector)
		
	}

	/**
	 * Handle behavior in INTAKE_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleIntakeState(TeleopInput input) {
		indexMotor.setVoltage(flywheelMotor.getInputVoltage());
		
	}

	/**
	 * Handle behavior in MANUAL_PREP_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleManualPrepState(TeleopInput input) {

		//FOR MANUAL ONLY: Right Bumper will be used as a deincrementer. Do not confuse this with triggering Passer Prep.
		//FOR MANUAL ONLY: Left Bumper will be used to adjust hood angle. Do not confuse this with triggering Shooter Prep.
		double hoodIncrementer = 5; //how much the hood angle increases/decreases each click
		if (input.isLeftBumperPressed() && input.isRightBumperPressed()){
			if (hoodTargetAngle - hoodIncrementer >= 45){
				hoodTargetAngle -= hoodIncrementer;
			} else {
				hoodTargetAngle = 45;
			}
			//decrease hood angle by 5 degrees
		} else if (input.isLeftBumperPressed()){
			if (hoodTargetAngle + hoodIncrementer <= 70){
				hoodTargetAngle += hoodIncrementer;
			} else {
				hoodTargetAngle = 70;
			}
			//increase hood angle by 5 degrees
		}
		updateHood();

		double flywheelIncrementer = 10; //how much the flywheel speed increases/decreases each click
		if (input.isLeftTriggerPressed() && input.isRightBumperPressed()){
			if (flywheelTargetSpeed - flywheelIncrementer > 0){
				flywheelTargetSpeed -= flywheelIncrementer;
			} else {
				flywheelTargetSpeed = 0;
			}
			//decrease flywheel speed by some constant, right now set to 10 m/s
		} else if (input.isLeftTriggerPressed()){
			if (flywheelTargetSpeed + flywheelIncrementer < ShooterConstants.FLYWHEEL_MAX_SPEED){
				flywheelTargetSpeed += flywheelIncrementer;
			} else {
				flywheelTargetSpeed = ShooterConstants.FLYWHEEL_MAX_SPEED;
			}
			
			//increase flywheel speed by some constant, right now set to 10 m/s
		}

		updateFlywheel();

		//check if current speed of motors and current angle matches what we just set it to there with the boolean conditions
		
	}

	private void updateFlywheel(){
		MotionMagicVelocityVoltage flywheel_request = new MotionMagicVelocityVoltage(0);
		flywheelMotor.setControl(flywheel_request.withVelocity(flywheelTargetSpeed));
		flywheelSpeed = flywheelMotor.getVelocity().getValue().in(Units.RotationsPerSecond);
	}

	private void updateHood(){
		MotionMagicVoltage hood_request = new MotionMagicVoltage(0);
		hoodMotor.setControl(hood_request.withPosition(hoodTargetAngle));
		hoodAngle = hoodMotor.getPosition().getValue().in(Units.Degrees);
	}


	/**
	 * Performs action for auto STATE1.
	 * @return if the action carried out has finished executing
	 */
	private boolean handleAutoState1() {
		return true;
	}

	/**
	 * Performs action for auto STATE2.
	 * @return if the action carried out has finished executing
	 */
	private boolean handleAutoState2() {
		return true;
	}

	/**
	 * Performs action for auto STATE3.
	 * @return if the action carried out has finished executing
	 */
	private boolean handleAutoState3() {
		return true;
	}
}