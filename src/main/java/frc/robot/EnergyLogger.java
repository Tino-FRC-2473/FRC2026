package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

public class EnergyLogger {
	private PowerDistribution pdp;


	/**
	 * Construct an energy logger.
	 */
	public EnergyLogger() {
		pdp = new PowerDistribution(HardwareMap.CAN_ID_POWER_DISTRIBUTION_HUB, ModuleType.kRev);
		System.out.println("EnergyLogger initialized on " + pdp.getType() + " with "
			+ pdp.getVersion() + " version");


	}


	/**
	 * Log energy data to the logger.
	 */
	public void log() {
		double totalCurrent = pdp.getTotalCurrent();

		Logger.recordOutput("Energy/TotalCurrent", totalCurrent);
		PowerDistributionFaults faults = pdp.getFaults();
		double voltage = pdp.getVoltage();

		for (int i = 0; i < pdp.getNumChannels(); i++) {
			double current = pdp.getCurrent(i);
			double power = current * voltage;
			if (faults.getBreakerFault(i)) {
				Logger.recordOutput("Energy/Channel" + i + "/Fault", true);
			} else {
				Logger.recordOutput("Energy/Channel" + i + "/Fault", false);
			}
			Logger.recordOutput("Energy/Channel" + i + "/Voltage", voltage);
			Logger.recordOutput("Energy/Channel" + i + "/Current", current);
			Logger.recordOutput("Energy/Channel" + i + "/Power", power);
		}
	}


}


