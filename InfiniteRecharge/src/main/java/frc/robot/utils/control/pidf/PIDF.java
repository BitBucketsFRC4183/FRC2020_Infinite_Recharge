package frc.robot.utils.control.pidf;



public class PIDF extends PID {
    protected final double KF;


    
    /**
     * Generate a set of PID constants
     * 
     * @param kP proportional constant
     * @param kI integral constant
     * @param kD derivative constant
     * @param kF feed-forward constant
     */
    public PIDF(double kP, double kI, double kD, double kF) {
        super(kP, kI, kD);
        
        KF = kF;
    }

    /**
     * Generate a set of PID constants
     * 
     * @param kP proportional constant
     * @param kI integral constant
     * @param kD derivative constant
     * @param kF feed-forward constant
     * @param iZone integral zone
     */
    public PIDF(double kP, double kI, double kD, double kF, double iZone) {
        this(kP, kI, kD, kF);

        this.iZone = iZone;
    }

    

    public double getKF() { return KF; }
}