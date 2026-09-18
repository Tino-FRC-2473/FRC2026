package frc.robot.motors;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Robot;

/**
 * Updated SparkMaxWrapper for 2026.
 * Implements the new declarative configuration API and LoggedMotor interface.
 */
public class SparkMaxWrapper extends SparkMax implements LoggedMotor {

    private final DCMotor motorPlant;
    private final SparkMaxSim motorSim;
    
    // Config object for 2026 API
    private final SparkMaxConfig config;

    // WPILib standard loop is 20ms (0.020s)
    public static final double LOOP_PERIOD_SECONDS = 0.020;

    private double targetVelocityRadPerSec = 0;

	public SparkMaxWrapper(int deviceId, MotorType type, DCMotor motorPlant) {
        super(deviceId, type);
        
        this.motorPlant = motorPlant;
        this.config = new SparkMaxConfig();

      
        config.smartCurrentLimit(40)
              .idleMode(SparkMaxConfig.IdleMode.kBrake);

        this.configure(config, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);


        // 3. Initialize Sim
        this.motorSim = new SparkMaxSim(this, motorPlant);


        init();
    }

    @Override
    public void updateSimState() {
        // Iterate physics: (applied_velocity, bus_voltage, dt)
        motorSim.iterate(targetVelocityRadPerSec, RobotController.getBatteryVoltage(), LOOP_PERIOD_SECONDS);
    }

    @Override
    public void set(double speed) {
        super.set(speed);
        this.targetVelocityRadPerSec = speed * motorPlant.freeSpeedRadPerSec;
    }

    @Override
    public String getIdentifier() {
        return Integer.toString(this.getDeviceId());
    }

    @Override
	public double getLoggedPosition() {
		// Sim motor
		if (Robot.isSimulation()) {
			return motorSim.getPosition();
		}

		// Real motor
		return this.getEncoder().getPosition();
	}

	@Override
	public double getLoggedVelocity() {
		// Sim motor
		if (Robot.isSimulation()) {
			return motorSim.getVelocity();
		}

		// Real motor
		return this.getEncoder().getVelocity();
	}

	@Override
	public double getLoggedSetpoint() {
		return motorSim.getSetpoint();
	}

	@Override
	public double getLoggedVoltage() {
		return motorSim.getBusVoltage();
	}

	@Override
	public double getLoggedCurrent() {
		return motorSim.getMotorCurrent();
	}
}