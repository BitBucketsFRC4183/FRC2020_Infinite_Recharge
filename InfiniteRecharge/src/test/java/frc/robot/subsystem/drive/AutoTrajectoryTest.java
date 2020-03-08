package frc.robot.subsystem.drive;

import java.util.List;

import org.junit.Test;

import static org.junit.Assert.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.subsystem.drive.auto.FieldConstants;

public class AutoTrajectoryTest {
    @Test
    public void autoTrajTest() throws InterruptedException {
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            1,
            1
        );

        Trajectory autoTrajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(FieldConstants.START_CENTER_POWER_PORT, Rotation2d.fromDegrees(90)),
            List.of(),//FieldConstants.OUR_POWER_CELL_1, FieldConstants.OUR_POWER_CELL_2),
            new Pose2d(FieldConstants.OUR_POWER_CELL_1, Rotation2d.fromDegrees(90)),
            trajectoryConfig
        );

        for (int i = 0; i < 50; i++) {
            double time = 0.02*i;

            System.out.println(time + ": " + autoTrajectory.sample(time).poseMeters.getTranslation().toString());
        }
    }
}