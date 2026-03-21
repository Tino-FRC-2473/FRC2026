package frc.robot.util;

import edu.wpi.first.wpilibj.RobotController;
import java.util.HashMap;
import java.util.Map;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

/** Utility class for logging energy usage. */
public class EnergyLogger {
  private static double totalCurrent = 0.0;
  private static double totalPower = 0.0;
  private static double totalEnergy = 0.0;

  private static Map<String, Double> subsystemCurrents = new HashMap<>();
  private static Map<String, Double> subsystemPowers = new HashMap<>();
  private static Map<String, Double> subsystemEnergies = new HashMap<>();

  private static BatteryIOInputsAutoLogged inputs = new BatteryIOInputsAutoLogged();

  public static void updateBatteryVoltage() {
    inputs.batteryVoltage = RobotController.getBatteryVoltage();
    inputs.rioCurrent = RobotController.getInputCurrent();
    Logger.processInputs("EnergyLogger", inputs);
  }

  public static void recordEnergyUsage(String key, double... amps) {
    double totalAmps = 0.0;
    for (int i = 0; i < amps.length; i++) {
totalAmps += amps[i];
    }

    double power = totalAmps * inputs.batteryVoltage;
    double energy = power * Constants.loopPeriodSecs;

    totalCurrent += totalAmps;
    totalPower += power;
    totalEnergy += energy;

    subsystemCurrents.put(key, totalAmps);
    subsystemPowers.put(key, power);
    subsystemEnergies.put(key, subsystemEnergies.getOrDefault(key, 0.0) + energy);

    String[] keys = key.split("/");
    if (keys.length < 2) {
return;
    }

    String subkey = "";
    for (int i = 0; i < keys.length - 1; i++) {
subkey += keys[i];
if (i < keys.length - 2) {
  subkey += "/";
}
subsystemCurrents.put(subkey, subsystemCurrents.getOrDefault(subkey, 0.0) + totalAmps);
subsystemPowers.put(subkey, subsystemPowers.getOrDefault(subkey, 0.0) + power);
subsystemEnergies.put(subkey, subsystemEnergies.getOrDefault(subkey, 0.0) + energy);
    }
  }

  public static void recordOutputs() {
    recordEnergyUsage("Controls/roboRIO", inputs.rioCurrent);
    // Add other constant loads if known (CANcoders, Pigeon, etc.)
    recordEnergyUsage("Controls/CANcoders", 0.05 * 4);
    recordEnergyUsage("Controls/Pigeon", 0.04);
    recordEnergyUsage("Controls/Radio", 0.5);

    Logger.recordOutput("EnergyLogger/Current", totalCurrent);
    Logger.recordOutput("EnergyLogger/Power", totalPower);
    Logger.recordOutput("EnergyLogger/Energy", joulesToWattHours(totalEnergy));

    for (var entry : subsystemCurrents.entrySet()) {
Logger.recordOutput("EnergyLogger/Current/" + entry.getKey(), entry.getValue());
subsystemCurrents.put(entry.getKey(), 0.0);
    }
    for (var entry : subsystemPowers.entrySet()) {
Logger.recordOutput("EnergyLogger/Power/" + entry.getKey(), entry.getValue());
subsystemPowers.put(entry.getKey(), 0.0);
    }
    for (var entry : subsystemEnergies.entrySet()) {
Logger.recordOutput(
    "EnergyLogger/Energy/" + entry.getKey(),
    joulesToWattHours(entry.getValue()));
    }

    totalPower = 0.0;
    totalCurrent = 0.0;
  }

  private static double joulesToWattHours(double joules) {
    return joules / 3600.0;
  }

  @AutoLog
  public static class BatteryIOInputs {
    public double batteryVoltage = 12.0;
    public double rioCurrent = 0.0;
  }
}
