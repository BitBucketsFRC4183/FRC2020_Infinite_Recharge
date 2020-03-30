package frc.robot.subsystem.drive.auto;

import edu.wpi.first.wpilibj.geometry.Translation2d;

/**
 * Constants of where multiplle points are on the map
 * Units in M, calculated from the center of the field
 */
public class FieldConstants {
    public static final double INITIALIZATION_LINE = 5;

    // starting points
    public static final Translation2d START_CENTER_POWER_PORT = new Translation2d(
        1.5, INITIALIZATION_LINE
    );

    public static final Translation2d START_OPPONENT_TRENCH = new Translation2d(
        -3.5, INITIALIZATION_LINE
    );

    public static final Translation2d START_OUR_TRENCH = new Translation2d(
        2.8, INITIALIZATION_LINE
    );

    // power cell points

    public static final Translation2d BEFORE_OUR_POWER_CELL_1 = new Translation2d(
        3, 1.4
    );
    public static final Translation2d ALSO_BEFORE_OUR_POWER_CELL_1 = new Translation2d(
        2.9, 1.39
    );

    public static final Translation2d OUR_POWER_CELL_1 = new Translation2d(
        2.9, 1.83
    );

    public static final Translation2d OUR_POWER_CELL_2 = new Translation2d(
        2.9, 0.91
    );

    public static final Translation2d OUR_POWER_CELL_3 = new Translation2d(
        2.9, -0.3
    );

    public static final Translation2d OUR_TWO_POWER_CELLS = new Translation2d(
        2.9, -1.5
    );

    public static final Translation2d THEIR_TWO_POWER_CELLS = new Translation2d(
        -3.5, 0.75
    );

    /** Use the coordinate system the robot likes
     * @param pointToGo The point where you want to move to; most likely one of the POWER_CELL points in the class
     * @param startingPoint Where you're starting on the initiation line, will be one of the START_ points in the class
    */
    public static Translation2d transformToRobot(Translation2d pointToGo, Translation2d startingPoint) {
        Translation2d temp = pointToGo.minus(startingPoint);

        return new Translation2d(-temp.getY(), temp.getX());
    }
}