package frc.robot.auto;

import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import java.lang.annotation.ElementType;

@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.METHOD)
public @interface DashboardChoosable {

/**
 * The name that will appear on the SmartDashboard chooser.
 *
 * @return the display name for the autonomous routine
 */
	String key();
}
