package frc.robot.subsystem.vision;

public class VisionConstants {

    static final double TARGET_HEIGHT_INCHES = 98.25;
    static final double CAMERA_HEIGHT_INCHES = 22; // approx (unknown for now)
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