package frc.robot.config;

import frc.robot.utils.control.pidf.PIDF;

public class MotorConfig {

    //////////////////////////////////////////////////////////////////////////////
    // PIDFs

    // public class PID{
    //     public float kp;
    //     public float ki;
    //     public float kd;
    //     public float kf;
    // }

    public PIDF velocityPIDF = new PIDF();

    public PIDF positionPIDF;

    //////////////////////////////////////////////////////////////////////////////
    // ID

    public int id;
}