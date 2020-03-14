package frc.robot.utils.dashboard.annotations;

import java.lang.annotation.*;

@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.TYPE)
@Inherited
public @interface Dashboard {

    /**
     * This is used to build the name of the dashboard
     * @return
     */
    String value() default "";

    /**
     * The time to delay refreshing values when getting from the smart dashboard, in seconds.
     * for example, if this is set to 1, it will return the same value from the SmartDashboard for
     * 1 second, no matter how many times the get methods are called, before querying the SmartDasboard again
     * @return
     */
    int getDelaySeconds() default 0;

    /**
     * Set to true to populate any getter values in the dashboard when this Dashboard instance is created
     * The values will be populated with the defined default
     * @return
     */
    boolean populateOnCreate() default true;
}
