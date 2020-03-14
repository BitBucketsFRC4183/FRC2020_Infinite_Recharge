package frc.robot.utils.dashboard.annotations;

import java.lang.annotation.*;

/**
 * The Key of the smartdashboard entry
 */
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.METHOD)
@Inherited
public @interface Key {
    String value();
    boolean exactPath() default false;
}
