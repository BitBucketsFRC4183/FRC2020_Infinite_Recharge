package frc.robot.subsystem.drive.auto.commands;

import frc.robot.subsystem.drive.auto.FieldConstants;
import frc.robot.subsystem.drive.auto.FullTrajectory;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import java.util.List;


public class CenterThreeBall {

    Trajectory firstPickup;
    Trajectory firstReturn;

    /**
     * auto trajectory for if we start at the center of the initiation line
     * @param config TrajectoryConfig with parameters and constraints added
     */
    public CenterThreeBall(TrajectoryConfig config) {

        Translation2d startingPoint = FieldConstants.START_CENTER_POWER_PORT;
        firstPickup = TrajectoryGenerator.generateTrajectory(
            // Start in front of the power port
            new Pose2d(
                FieldConstants.transformToRobot(FieldConstants.START_CENTER_POWER_PORT, startingPoint),
                new Rotation2d()
            ),
            // Pass through these two interior waypoints
            List.of(
                FieldConstants.transformToRobot(FieldConstants.OUR_POWER_CELL_1, startingPoint),
                FieldConstants.transformToRobot(FieldConstants.OUR_POWER_CELL_2, startingPoint)
            ),
            // End at where the third ball is
            new Pose2d(
                FieldConstants.transformToRobot(FieldConstants.OUR_POWER_CELL_3, startingPoint),
                new Rotation2d()
            ),
            // we are not reversed and we end at a velocity of 0
            config.setReversed(false).setEndVelocity(0)
        );
        firstReturn = TrajectoryGenerator.generateTrajectory(
            // Start at where the third ball is
            new Pose2d(
                FieldConstants.transformToRobot(FieldConstants.OUR_POWER_CELL_3, startingPoint),
                new Rotation2d()
            ),
            List.of(),
            // Return to the power port
            new Pose2d(
                FieldConstants.transformToRobot(FieldConstants.START_CENTER_POWER_PORT, startingPoint),
                new Rotation2d()
            ),
            // // we ARE reversed and we end at a velocity of 0
            config.setReversed(true).setEndVelocity(0)
        );

    }

    public FullTrajectory getFullTrajectory() {
        return new FullTrajectory("center", firstPickup, firstReturn);
    }
    
}