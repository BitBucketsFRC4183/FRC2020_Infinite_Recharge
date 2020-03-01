package frc.robot.subsystem.vision;

public class VisionConstants {

    static final double TARGET_HEIGHT_INCHES = 89.75;//81.0;//82.25;//89.75;//98.25;//82.25;// 98.25;// for the absolute top part of the tape
    static final double CAMERA_HEIGHT_INCHES = 22.6;
    static final double CAMERA_MOUNTING_ANGLE = 30;
    
    static final double BALL_SHOOTING_ANGLE = 30; // wild guess

    public static double getTargetHeightInches() {
        return TARGET_HEIGHT_INCHES;
    }

    public static double getCameraHeightInches() {
        return CAMERA_HEIGHT_INCHES;
    }

    public static double getCameraMountingAngle() {
        return CAMERA_MOUNTING_ANGLE;
    }
}