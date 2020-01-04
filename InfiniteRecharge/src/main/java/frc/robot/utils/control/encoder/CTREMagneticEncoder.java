package frc.robot.utils.control.encoder;

public class CTREMagneticEncoder extends SensorType {
    /*
     * CTRE supports both relative and absolute magnetic encoders,
     * but a relative can function as an absolute so no need
     */
    public CTREMagneticEncoder() {
        super(4096);
    }
}