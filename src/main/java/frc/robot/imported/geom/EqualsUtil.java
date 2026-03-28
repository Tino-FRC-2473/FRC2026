package frc.robot.imported.geom;

public class EqualsUtil {

	/**
	 * Returns if difference between two numbers is between margin of error.
	 * @param a first number
	 * @param b second number
	 * @param moe margin of error
	 * @return is the difference within margin of error
	 */
	public static boolean epsilonEquals(double a, double b, double moe) {
		return (((b > a) ? (b - a) <= moe : (a - b) <= moe));
	}
}
