package frc.robot.subsystem.vision;

public class VisionConstants {


    // height of the target from the ground
    // there are a bunch of different numbers since we were messing with this number to get the distance to work
    static final double TARGET_HEIGHT_INCHES = 89.75;//ignore these-> //81.0;//82.25;//89.75;//98.25;//82.25;// 98.25;// for the absolute top part of the tape
    // height of the camera from above the ground
    static final double CAMERA_HEIGHT_INCHES = 22.6;
    // cemra mounting angle
    static final double CAMERA_MOUNTING_ANGLE = 30;

    // what angle the ball shoots at
    // not used anywhere
    // and also wrong? since we control the angle of the shot
    static final double BALL_SHOOTING_ANGLE = 30; // wild guess

    // bool to use/not use running average filter for calculating the tx
    static final boolean USE_FILTER = false;
    // how many values the filter should use
    static final int FILTER_LENGTH = 25;

    // flag to use/not use the auto-pipeline-switching-to-adjust-zoom-based-on-distance feature
    static final boolean ENABLE_AUTO_ZOOM = false;

    // We have a tx bias since the drive auto-align requires it
    static final double TX_BIAS_DEG = Math.PI; // about 3.1, so why not

    // getteeeeeeeeeeeeeeers
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