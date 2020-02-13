package frc.robot.subsystem.drive;

import edu.wpi.first.wpilibj.geometry.Translation2d;

// all units in m
public class FieldConstants {
    static final double INITIALIZATION_LINE = 4.943475000000000;



    static final Translation2d FRONT_OF_POWER_PORT = new Translation2d(
        1.700784, INITIALIZATION_LINE
    );



    static final Translation2d OUR_POWER_CELL_1 = new Translation2d(
        3.400298, 1.828673
    );

    static final Translation2d OUR_POWER_CELL_2 = new Translation2d(
        3.400298, 0.914273
    );

    static final Translation2d OUR_POWER_CELL_3 = new Translation2d(
        3.400298, -0.000127
    );
}