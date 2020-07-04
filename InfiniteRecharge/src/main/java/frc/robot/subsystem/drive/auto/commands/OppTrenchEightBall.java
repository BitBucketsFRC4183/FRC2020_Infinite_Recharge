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


public class OppTrenchEightBall {

    Trajectory firstPickup;
    Trajectory firstReturn;
    Trajectory secondPickup;
    Trajectory secondReturn;

    /**
     * auto trajectory for if we start at the opponent's trench
     * @param config TrajectoryConfig with parameters and constraints added
     */
    public OppTrenchEightBall(TrajectoryConfig config) {

        Translation2d startingPoint = FieldConstants.START_OPPONENT_TRENCH;
        firstPickup = TrajectoryGenerator.generateTrajectory(
            new Pose2d(
                FieldConstants.transformToRobot(FieldConstants.START_OPPONENT_TRENCH, startingPoint),
                new Rotation2d()
            ),
            List.of(),
            new Pose2d(
                FieldConstants.transformToRobot(FieldConstants.THEIR_TWO_POWER_CELLS, startingPoint),
                new Rotation2d()
            ),
            config.setReversed(false).setEndVelocity(0)
        );
        firstReturn = TrajectoryGenerator.generateTrajectory(
            new Pose2d(
                FieldConstants.transformToRobot(FieldConstants.THEIR_TWO_POWER_CELLS, startingPoint),
                new Rotation2d()
            ),
            List.of(),
            new Pose2d(
                FieldConstants.transformToRobot(FieldConstants.START_CENTER_POWER_PORT, startingPoint),
                new Rotation2d()
            ),
            config.setReversed(true).setEndVelocity(0)
        );

        secondPickup = TrajectoryGenerator.generateTrajectory(
            new Pose2d(
                FieldConstants.transformToRobot(FieldConstants.START_CENTER_POWER_PORT, startingPoint),
                new Rotation2d()
            ),
            List.of(
                FieldConstants.transformToRobot(FieldConstants.OUR_POWER_CELL_1, startingPoint),
                FieldConstants.transformToRobot(FieldConstants.OUR_POWER_CELL_2, startingPoint)
            ),
            new Pose2d(
                FieldConstants.transformToRobot(FieldConstants.OUR_POWER_CELL_3, startingPoint),
                new Rotation2d()
            ),
            config.setReversed(false).setEndVelocity(0)
        );
        secondReturn = TrajectoryGenerator.generateTrajectory(
            new Pose2d(
                FieldConstants.transformToRobot(FieldConstants.OUR_POWER_CELL_3, startingPoint),
                new Rotation2d()
            ),
            List.of(),
            new Pose2d(
                FieldConstants.transformToRobot(FieldConstants.START_CENTER_POWER_PORT, startingPoint),
                new Rotation2d()
            ),
            config.setReversed(true).setEndVelocity(0)
        );

    }

    public FullTrajectory getFullTrajectory() {
        return new FullTrajectory("opponent trench", firstPickup, firstReturn, secondPickup, secondPickup);
    }
    
}