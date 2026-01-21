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
	SHOOTER_INTAKE_STATE,
	PASSER_INTAKE_STATE,
	MANUAL_PREP_STATE,
	MANUAL_INTAKE_STATE
}

public class ShooterFSMSystem extends FSMSystem<FSMState> {
	/* ======================== Constants ======================== */

	private static final float MOTOR_RUN_POWER = 0.1f;
	

	/* ======================== Private variables ======================== */

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private FSMState curState;
	//for curPose, we need to find the height of the shooter from the floor for calculations.
	private Pose2d curPose;
	private TalonFX flywheelMotor;
	private TalonFX indexMotor;
	private ShooterHoodSystem hood;
	private boolean flywheelAtSpeed;
	private boolean hoodAtAngle;


	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public ShooterFSMSystem(){
		curPose = new Pose2d();
		flywheelMotor = new TalonFXWrapper(
			HardwareMap.CAN_ID_FLYWHEEL
		);
		indexMotor = new TalonFXWrapper(
			HardwareMap.CAN_ID_INDEX
		);
		hood = new ShooterHoodSystem();
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

	@Override
	public void reset() {
		setCurrentState(FSMState.IDLE_STATE);

		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}

	@Override
	public void update(TeleopInput input) {
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

			case SHOOTER_INTAKE_STATE:
				handleShooterIntakeState(input);
				break;

			case PASSER_INTAKE_STATE:
				handlePasserIntakeState(input);
				break;

			case MANUAL_PREP_STATE:
				handleManualPrepState(input);
				break;
			
			case MANUAL_INTAKE_STATE:
				handleManualIntakeState(input);
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
					return FSMState.PASSER_PREP_STATE;
				} else if (input != null && input.isLeftBumperPressed()) {
					return FSMState.MANUAL_PREP_STATE;
				}

			case PASSER_PREP_STATE:

				if (input.isTouchpadPressed()){
					return FSMState.IDLE_STATE;
				}
				if (input.isRightBumperPressed()){
					return FSMState.SHOOTER_PREP_STATE;
				} 
				if (input.isLeftBumperPressed()){
					return FSMState.MANUAL_PREP_STATE;
				}
				if (flywheelAtSpeed && hoodAtAngle && input.isRightTriggerPressed()){
					return FSMState.PASSER_INTAKE_STATE;
				}

			case PASSER_INTAKE_STATE:
				if (!flywheelAtSpeed || !hoodAtAngle || !input.isRightTriggerPressed()){
					return FSMState.PASSER_PREP_STATE;
				}

				if (input.isTouchpadPressed()){
					return FSMState.IDLE_STATE;
				}

				if (input.isRightBumperPressed()){
					return FSMState.SHOOTER_PREP_STATE;
				}

				if (input.isLeftBumperPressed()){
					return FSMState.MANUAL_PREP_STATE;
				}
			
			case SHOOTER_PREP_STATE:
				if (input.isTouchpadPressed()){
					return FSMState.IDLE_STATE;
				}
				if (input.isRightBumperPressed()){
					return FSMState.SHOOTER_PREP_STATE;
				} 
				if (input.isLeftBumperPressed()){
					return FSMState.MANUAL_PREP_STATE;
				}
				if (flywheelAtSpeed && hoodAtAngle && input.isRightTriggerPressed()){
					return FSMState.PASSER_INTAKE_STATE;
				}
			
			case SHOOTER_INTAKE_STATE:
				if (!flywheelAtSpeed || !hoodAtAngle || !input.isRightTriggerPressed()){
					return FSMState.SHOOTER_PREP_STATE;
				}

				if (input.isTouchpadPressed()){
					return FSMState.IDLE_STATE;
				}

				if (input.isRightBumperPressed()){
					return FSMState.PASSER_PREP_STATE;
				}

				if (input.isLeftBumperPressed()){
					return FSMState.MANUAL_PREP_STATE;
				}

			case MANUAL_PREP_STATE:
				if (input.isTouchpadPressed()){
					return FSMState.IDLE_STATE;
				}
			
				if (input.isRightBumperPressed()){
					return FSMState.PASSER_PREP_STATE;
				}

				if (flywheelAtSpeed && hoodAtAngle && input.isRightTriggerPressed()){
					return FSMState.MANUAL_INTAKE_STATE;
				}
			
			case MANUAL_INTAKE_STATE:
				if (!flywheelAtSpeed || !hoodAtAngle || !input.isRightTriggerPressed() || input.isLeftBumperPressed()){
					return FSMState.MANUAL_PREP_STATE;
				}
				if (input.isTouchpadPressed()){
					return FSMState.IDLE_STATE;
				}
				if (input.isRightBumperPressed()){
					return FSMState.PASSER_PREP_STATE;
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
		hood.setAngle(0);

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
	 * Handle behavior in PASSER_INTAKE_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handlePasserIntakeState(TeleopInput input) {
		//turn on indexer motor to some value based off of the other values
		
	}

	/**
	 * Handle behavior in SHOOTER_INTAKE_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleShooterIntakeState(TeleopInput input) {
		//turn on indexer motor to some value based off of the other values
		
	}

	/**
	 * Handle behavior in MANUAL_PREP_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleManualPrepState(TeleopInput input) {
		if (input.isLeftBumperPressed()){
			//increase hood angle by 5 degrees
		}
		if (input.isLeftTriggerPressed()){
			//increase flywheel speed by some constant
		}

		//check if current speed of motors and current angle matches what we just set it to there with the boolean conditions
		
	}

	/**
	 * Handle behavior in MANUAL_INTAKE_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleManualIntakeState(TeleopInput input) {
		//turn on indexMotor to some constant based on the other values
		
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

	public void setHoodMotor(double hoodSpeed){
		hoodMotor.set(hoodSpeed);
	}

	public void setAngle(double angle){
		//functionality to be added, this will handle actually moving the hood
	}

}
