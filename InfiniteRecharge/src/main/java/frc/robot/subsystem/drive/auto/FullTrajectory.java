/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystem.drive.auto;

import java.util.Collections;
import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
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
    private final boolean hasSecondTrajectory;

    private FullTrajectory(String name, Trajectory firstPickupTrajectory, Trajectory firstReturnTrajectory,
            Trajectory secondPickupTrajectory, Trajectory secondReturnTrajectory, boolean hasSecondTrajectory) {
        this.name = name;
        this.firstPickupTrajectory = firstPickupTrajectory;
        this.firstReturnTrajectory = firstReturnTrajectory;
        this.secondPickupTrajectory = secondPickupTrajectory;
        this.secondReturnTrajectory = secondReturnTrajectory;
        this.hasSecondTrajectory = hasSecondTrajectory;
    }

    public FullTrajectory(String name, Trajectory pickupT, Trajectory returnT) {
        this(name,
                pickupT,
                returnT,
                null,
                null,
                false
        );
    }

    public FullTrajectory(String name, Trajectory firstPickupTrajectory, Trajectory firstReturnTrajectory,
            Trajectory secondPickupTrajectory, Trajectory secondReturnTrajectory) {
        this(name, firstPickupTrajectory, firstReturnTrajectory, secondPickupTrajectory, secondReturnTrajectory, true);
    }
    

    public String getName() {
        return name;
    }

    public Trajectory getFirstPickupTrajectory() {
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

    public boolean hasSecondTrajectory() {
        return hasSecondTrajectory;
    }

}
