/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystem.drive.auto;

import edu.wpi.first.wpilibj.trajectory.Trajectory;

/**
 * Add your docs here.
 */
public class FullTrajectory {

    private final String name;
    private final Trajectory pickupTrajectory;
    private final Trajectory returnTrajectory;

    public FullTrajectory(String name, Trajectory pickupT, Trajectory returnT) {
        this.name = name;
        pickupTrajectory = pickupT;
        returnTrajectory = returnT;
    }

    public String getName() {
        return name;
    }

    public Trajectory getPickupTrajectory(){
        return pickupTrajectory;
    }

    public Trajectory getReturnTrajectory() {
        return returnTrajectory;
    }

    

}
