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

// Third party Hardware Imports
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.BaseStatusSignal;

import java.util.ArrayList;
import java.util.List;


// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.motors.SparkMaxWrapper;
import frc.robot.motors.TalonFXWrapper;
import frc.robot.HardwareMap;
import frc.robot.systems.AutoHandlerSystem.AutoFSMState;
import frc.robot.systems.DriveFSMSystem;


enum FSMState {
	IDLE_STATE,
	SHOOTER_PREP_STATE,
	PASSER_PREP_STATE,
	INTAKE_STATE,
	MANUAL_PREP_STATE,
}

public class ShooterFSMSystem extends FSMSystem<FSMState> {
	//IMPORTANT NOTE: THIS WILL NOTE BUILD AS THIS IS JUST A FRAMEWORK AND DOES NOT HAVE ANY DEFINED INPUT FUNCTIONS OR CLASSES YET THAT WOULD BE PRESENT IN THE FULL CODE.
	/* ======================== Constants ======================== */

	private static final float MOTOR_RUN_POWER = 0.1f;
	

	/* ======================== Private variables ======================== */

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private FSMState curState;
	//for curPose, we need to find the height of the shooter from the floor for calculations.
	private Pose2d curPose;
	private Pose3d outpostPose;
	private Pose3d hubPose;
	private Pose3d target3Pose; //probably going to be the mirrored side of the outpost
	private List<Pose3d> targetPoses;
	private TalonFX flywheelMotor;
	private TalonFX indexMotor;
	private ShooterHoodSystem hood;
	private double flywheelSpeed;
	private double flywheelTargetSpeed;
	private double hoodAngle;
	private double hoodTargetAngle;
	private FSMState pastState;
	

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public ShooterFSMSystem(){
		curPose = new Pose2d();
		flywheelMotor = new TalonFXWrapper(
			HardwareMap.CAN_ID_FLYWHEEL //not made yet
		);
		indexMotor = new TalonFXWrapper(
			HardwareMap.CAN_ID_INDEX //not made yet
		);
		hood = new ShooterHoodSystem();
		reset();
	}
	
	public ShooterFSMSystem(DriveFSMSystem driveSystem) {
		// Perform hardware init using a wrapper class
		// this is so we can see motor outputs during simulatiuons
		curPose = driveSystem.getPose();
		flywheelMotor = new TalonFXWrapper(
			HardwareMap.CAN_ID_FLYWHEEL
		);
		indexMotor = new TalonFXWrapper(
			HardwareMap.CAN_ID_INDEXER
		);
		hood = new ShooterHoodSystem();
		// Reset state machine
		reset();
	}

	/* ======================== Public methods ======================== */

	// overridden methods don't require javadocs
	// however, you may want to add implementation specific javadocs

	public boolean isAtSpeed(){
		return (flywheelSpeed == flywheelTargetSpeed);
	}

	public boolean isAtAngle(){
		return (hoodAngle == hoodTargetAngle);
	}
	
	@Override
	public void reset() {
		setCurrentState(FSMState.IDLE_STATE);

		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}

	@Override
	public void update(TeleopInput input) {
		Logger.recordOutput("curState", curState);
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
					pastState = curState;
					curState = FSMState.PASSER_PREP_STATE;
					return curState;
				} else if (input != null && input.isLeftBumperPressed()) {
					pastState = curState;
					curState = FSMState.SHOOTER_PREP_STATE;
					return curState;
				} else if (input != null && input.isLeftTriggerPressed()){
					pastState = curState;
					curState = FSMState.MANUAL_PREP_STATE;
					return curState;
				}

			case PASSER_PREP_STATE:
				if (input.isTouchpadPressed()){
					pastState = curState;
					curState = FSMState.IDLE_STATE;
					return curState;
				}
				if (input.isLeftBumperPressed()){
					pastState = curState;
					curState = FSMState.SHOOTER_PREP_STATE;
					return curState;
				} 
				if (input.isLeftTriggerPressed()){
					pastState = curState;
					curState = FSMState.MANUAL_PREP_STATE;
					return curState;
				}
				if (isAtSpeed() && isAtAngle() && input.isRightTriggerPressed()){ //need to make sure to change colors for if its at speed and at angle so that they know when to pull triggers
					pastState = curState;
					curState = FSMState.INTAKE_STATE;
					return curState;
				}

			case INTAKE_STATE:
				if (!isAtSpeed() || !isAtAngle() || !input.isRightTriggerPressed()){
					indexMotor.set(0);
					return pastState; //pastState should only store shooter_prep, passer_prep, and manual_prep
				}

				if (input.isTouchpadPressed()){
					pastState = curState;
					curState = FSMState.IDLE_STATE;
					return curState;
				}

				if (input.isRightBumperPressed()){
					pastState = curState;
					curState = FSMState.PASSER_PREP_STATE;
					return curState;
				}

				if (input.isLeftBumperPressed()){
					pastState = curState;
					curState = FSMState.SHOOTER_PREP_STATE;
					return curState;
				}

				if (input.isLeftTriggerPressed()){
					pastState = curState;
					curState = FSMState.MANUAL_PREP_STATE;
					return curState;
				}
			
			case SHOOTER_PREP_STATE:
				if (input.isTouchpadPressed()){
					pastState = curState;
					curState = FSMState.IDLE_STATE;
					return curState;
				}
				if (input.isRightBumperPressed()){
					pastState = curState;
					curState = FSMState.PASSER_PREP_STATE;
					return curState;
				} 
				if (input.isLeftTriggerPressed()){
					pastState = curState;
					curState = FSMState.MANUAL_PREP_STATE;
					return curState;
				}
				if (isAtSpeed() && isAtAngle() && input.isRightTriggerPressed()){
					pastState = curState;
					curState = FSMState.INTAKE_STATE;
					return curState;
				}

			case MANUAL_PREP_STATE:
			//Manual can only go to idle (we need the button inputs for right and left bumper to adjust manually)
				if (input.isTouchpadPressed()){
					pastState = curState;
					curState = FSMState.IDLE_STATE;
					return curState;
				}

				if (isAtSpeed() && isAtAngle() && input.isRightTriggerPressed()){
					pastState = curState;
					curState = FSMState.INTAKE_STATE;
					return curState;
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
		flywheelMotor.set(0);
		indexMotor.set(0);
		hood.setHoodAngle(20); //20 degrees from the vertical is the base angle
		//hood remains at current angle.

	}
	/**
	 * Handle behavior in PASSER_PREP_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handlePasserPrepState(TeleopInput input) {
		//TBD: code to find the distance vector from where we are to passing targets (preferably outpost and the location of outpost on the other side) (3d vector)
		
	}

	/**
	 * Handle behavior in SHOOTER_PREP_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleShooterPrepState(TeleopInput input) {
		//TBD: code to find the distance vector from where we are to hub center (3d vector)
		
	}

	/**
	 * Handle behavior in INTAKE_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleIntakeState(TeleopInput input) {
		indexMotor.set(flywheelTargetSpeed);
		
	}

	/**
	 * Handle behavior in MANUAL_PREP_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleManualPrepState(TeleopInput input) {

		//FOR MANUAL ONLY: Right Bumper will be used as a deincrementer. Do not confuse this with triggering Passer Prep.
		//FOR MANUAL ONLY: Left Bumper will be used to adjust hood angle. Do not confuse this with triggering Shooter Prep.
		if (input.isLeftBumperPressed() && input.isRightBumperPressed()){
			hood.setHoodAngle(hoodTargetAngle - 5);
			//decrease hood angle by 5 degrees
		} else if (input.isLeftBumperPressed()){
			hood.setHoodAngle(hoodTargetAngle + 5);
			//increase hood angle by 5 degrees
		}

		if (input.isLeftTriggerPressed() && input.isRightBumperPressed()){
			//decrease flywheel speed by some constant
		} else if (input.isLeftTriggerPressed()){
			//increase flywheel speed by some constant
		}

		//check if current speed of motors and current angle matches what we just set it to there with the boolean conditions
		
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

public class ShooterHoodSystem{
	private TalonFX hoodMotor; 

	public ShooterHoodSystem(){
		hoodMotor = new TalonFXWrapper(
			HardwareMap.CAN_ID_HOOD
		);
	}

	public TalonFX getHoodMotor(){
		return hoodMotor;
	}

	public void setHoodAngle(double hoodAngle){
		//functionality to be added
	}

}
