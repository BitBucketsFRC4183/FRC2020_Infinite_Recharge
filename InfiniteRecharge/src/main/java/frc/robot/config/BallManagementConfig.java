package frc.robot.config;

import frc.robot.config.MotorConfig;

public class BallManagementConfig {
    public static final double BMS_OUTPUT_PERCENT = 0.5;

    public MotorConfig spinner = new MotorConfig();

    public BallManagementConfig() {
        spinner.encoderType = MotorConfig.EncoderType.None;
        
    }
}