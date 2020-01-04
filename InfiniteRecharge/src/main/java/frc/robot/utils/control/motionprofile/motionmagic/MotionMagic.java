package frc.robot.utils.control.motionprofile.motionmagic;

import frc.robot.utils.math.units.Quantity;

public class MotionMagic {
    private final Quantity ACC;
    private final Quantity CRUISE_VELOCITY;

    public MotionMagic(Quantity acc, Quantity cruiseVelocity) {
        ACC = acc;
        CRUISE_VELOCITY = cruiseVelocity;
    }

    public Quantity getAcceleration() {
        return ACC;
    }

    public Quantity getCruiseVelocity() {
        return CRUISE_VELOCITY;
    }
}