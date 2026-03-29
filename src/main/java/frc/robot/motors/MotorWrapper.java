package frc.robot.motors;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.units.Units;

public class MotorWrapper {

    /**
     * Common motor interface for all motor types.
     */
    public interface MotorController {
        void setPercent(double output);

        void setVoltage(double volts);

        void setVelocity(double rpm);

        void setPosition(double rotations);

        double getPosition();

        double getVelocity();

        double getOutput();

        double getVoltage();

        double getCurrent();

        String getIdentifier();
    }

    /**
     * TalonFX implementation using composition.
     */
    public static class TalonFXMotor implements MotorController {

        private final TalonFX motor;

        // Control requests (reuse objects to avoid GC)
        private final DutyCycleOut dutyCycleRequest;
        private final VoltageOut voltageRequest;
        private final VelocityVoltage velocityRequest;
        private final PositionVoltage positionRequest;

        /**
         * Constructor with CAN ID.
         *
         * @param deviceId CAN ID of motor
         */
        public TalonFXMotor(int deviceId) {
            motor = new TalonFX(deviceId);

            dutyCycleRequest = new DutyCycleOut(0);
            voltageRequest = new VoltageOut(0);
            velocityRequest = new VelocityVoltage(0);
            positionRequest = new PositionVoltage(0);
        }

        @Override
        public void setPercent(double output) {
            motor.setControl(dutyCycleRequest.withOutput(output));
        }

        @Override
        public void setVoltage(double volts) {
            motor.setControl(voltageRequest.withOutput(volts));
        }

        @Override
        public void setVelocity(double rpm) {
            double rps = rpm / 60.0;
            motor.setControl(velocityRequest.withVelocity(rps));
        }

        @Override
        public void setPosition(double rotations) {
            motor.setControl(positionRequest.withPosition(rotations));
        }

        @Override
        public double getPosition() {
            return motor.getPosition().getValue().in(Units.Rotations);
        }

        @Override
        public double getVelocity() {
            return motor.getVelocity().getValue().in(Units.RPM);
        }

        @Override
        public double getOutput() {
            return motor.get();
        }

        @Override
        public double getVoltage() {
            return motor.getMotorVoltage().getValue().in(Units.Volts);
        }

        @Override
        public double getCurrent() {
            return motor.getSupplyCurrent().getValue().in(Units.Amps);
        }

        @Override
        public String getIdentifier() {
            return Integer.toString(motor.getDeviceID());
        }

        /**
         * Basic configuration (expand as needed).
         */
        public void configure(boolean inverted, boolean brakeMode) {

    var config = new com.ctre.phoenix6.configs.TalonFXConfiguration();

    // Inversion
    if (inverted) {
        config.MotorOutput.Inverted =
            com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive;
    } else {
        config.MotorOutput.Inverted =
            com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
    }

    // Neutral mode
    if (brakeMode) {
        config.MotorOutput.NeutralMode =
            com.ctre.phoenix6.signals.NeutralModeValue.Brake;
    } else {
        config.MotorOutput.NeutralMode =
            com.ctre.phoenix6.signals.NeutralModeValue.Coast;
    }

    motor.getConfigurator().apply(config);
}

        /**
         * Basic fault check.
         *
         * @return true if motor appears connected
         */
        public boolean isConnected() {
            return motor.getSupplyVoltage().getValue().in(Units.Volts) > 1.0;
        }
    }
}