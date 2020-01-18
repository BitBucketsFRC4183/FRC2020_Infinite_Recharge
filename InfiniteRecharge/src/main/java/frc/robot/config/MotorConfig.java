package frc.robot.config;

import frc.robot.utils.control.pidf.PIDF;

public class MotorConfig {

    //////////////////////////////////////////////////////////////////////////////
    // Other

    public int ticksPerRevolution;

    public boolean inverted;

    //////////////////////////////////////////////////////////////////////////////
    // PIDFs

    public PIDF velocityPIDF = new PIDF();

    public PIDF positionPIDF = new PIDF();

    //////////////////////////////////////////////////////////////////////////////
    // ID

    public int id;

    public int followingID = -1;
}