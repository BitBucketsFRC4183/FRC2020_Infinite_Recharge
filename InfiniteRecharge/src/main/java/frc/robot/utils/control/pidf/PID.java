package frc.robot.utils.control.pidf;



/**
 * Set of constants for a PID (proportional-integral-derivative) controller.
 * Typically used solely for position control (though MotionMagic is smoother)
 * or current control
 */
public class PID {
    // PID constants
    // these are protected so subclasses such as PIDF can access them easily
    protected final double KP;
    protected final double KI;
    protected final double KD;

    protected double iZone;

    private int uses = 0;

    

    /**
     * Generate a set of PID constants
     * 
     * @param kP proportional constant
     * @param kI integral constant
     * @param kD derivative constant
     */
    public PID(double kP, double kI, double kD) {
        KP = kP;
        KI = kI;
        KD = kD;
    }

    /**
     * Generate a set of PID constants
     * 
     * @param kP proportional constant
     * @param kI integral constant
     * @param kD derivative constant
     * @param iZone integral zone
     */
    public PID(double kP, double kI, double kD, double iZone) {
        this(kP, kI, kD);

        this.iZone = iZone;
    }



    public double getKP() { return KP; }
    public double getKI() { return KI; }
    public double getKD() { return KD; }
    public double getIZone() { return iZone; }

    public void use() {
        uses++;
    }

    public int getUses() {
        return uses;
    }
}