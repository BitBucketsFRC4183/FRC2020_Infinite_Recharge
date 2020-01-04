package frc.robot.utils.control.encoder;

/**
 * https://www.ctr-electronics.com/downloads/pdf/Talon%20SRX%20Software%20Reference%20Manual-1.pdf
 * 17.2.1. API requirements and Native Units for Unit Scaling
 */
public class SensorType {
    /** Ticks per revolution for this type of encoder */
    private final int TPR;
    
    public SensorType(int tpr) {
        TPR = tpr;
    }

    public int getTicksPerRev() {
        return TPR;
    }
}