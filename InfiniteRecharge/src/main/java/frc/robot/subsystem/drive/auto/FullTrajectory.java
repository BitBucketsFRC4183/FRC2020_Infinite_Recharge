/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystem.drive.auto;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;

/**
 * Add your docs here.
 */
public class FullTrajectory {

    private final String name;
    private final Trajectory firstPickupTrajectory;
    private final Trajectory firstReturnTrajectory;
    private final Trajectory secondPickupTrajectory;
    private final Trajectory secondReturnTrajectory;

    public FullTrajectory(String name, Trajectory firstPickupT, Trajectory firstReturnT, Trajectory secondPickupT, Trajectory secondReturnT) {
        this.name = name;
        firstPickupTrajectory = firstPickupT;
        firstReturnTrajectory = firstReturnT;
        secondPickupTrajectory = secondPickupT;
        secondReturnTrajectory = secondReturnT;
    }

    public FullTrajectory(String name, Trajectory pickupT, Trajectory returnT) {
        this(name,

            pickupT,

            returnT, 

            TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(), 
                new Pose2d(0, 0, new Rotation2d(0)),
                new TrajectoryConfig(1, 1)
            ), 

            TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(), 
                new Pose2d(0, 0, new Rotation2d(0)),
                new TrajectoryConfig(1, 1)
            )
        );
    }

    public String getName() {
        return name;
    }

    public Trajectory getFirstPickupTrajectory(){
        return firstPickupTrajectory;
    }

    public Trajectory getFirstReturnTrajectory() {
        return firstReturnTrajectory;
    }

    public Trajectory getSecondPickupTrajectory() {
        return secondPickupTrajectory;
    }

    public Trajectory getSecondReturnTrajectory() {
        return secondReturnTrajectory;
    }

    

}
