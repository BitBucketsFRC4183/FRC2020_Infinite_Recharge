package frc.robot.utils.math;

public class MathUtils {
    public static int round(double x) {
        if (x < 0) { return (int) (x - 0.5); }
        if (x > 0) { return (int) (x + 0.5); }

        return 0;
    }
}