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
 * FullTrajectory with a name, a first pickup and return, and sometimes a second pickup and return trajectory
 */
public class FullTrajectory {

    private final String name;
    private final Trajectory firstPickupTrajectory;
    private final Trajectory firstReturnTrajectory;
    private final Trajectory secondPickupTrajectory;
    private final Trajectory secondReturnTrajectory;
    private final boolean hasSecondTrajectory;

    /**
     * Constructor for both trajectories, also sets the 'hasSecondTrajectory' bool appropriately
     * @param name
     * @param firstPickupTrajectory
     * @param firstReturnTrajectory
     * @param secondPickupTrajectory
     * @param secondReturnTrajectory
     * @param hasSecondTrajectory
     */
    private FullTrajectory(String name, Trajectory firstPickupTrajectory, Trajectory firstReturnTrajectory,
            Trajectory secondPickupTrajectory, Trajectory secondReturnTrajectory, boolean hasSecondTrajectory) {
        this.name = name;
        this.firstPickupTrajectory = firstPickupTrajectory;
        this.firstReturnTrajectory = firstReturnTrajectory;
        this.secondPickupTrajectory = secondPickupTrajectory;
        this.secondReturnTrajectory = secondReturnTrajectory;
        this.hasSecondTrajectory = hasSecondTrajectory;
    }

    /**
     * Constructor for only 1 pickup and only 1 returnt trajectory
     * sets the 'hasSecondTrajectory' bool appropriately
     * @param name Name of trajectory
     * @param pickupT First pickup trajectory
     * @param returnT First return trajectory
     */
    public FullTrajectory(String name, Trajectory pickupT, Trajectory returnT) {
        this(name,
                pickupT,
                returnT,
                null,
                null,
                false
        );
    }

    /**
     * Constructor for both trajectories
     * @param name Name of trajectory
     * @param firstPickupTrajectory
     * @param firstReturnTrajectory
     * @param secondPickupTrajectory
     * @param secondReturnTrajectory
     */
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
