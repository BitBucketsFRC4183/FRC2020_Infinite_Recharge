package frc.robot.utils.control;

import frc.robot.utils.control.pidf.PID;
import frc.robot.utils.control.motionprofile.motionmagic.MotionMagic;
import frc.robot.utils.control.controltype.ControlType;

/*
 * ==================================
 * 2020 Code Review Party Information
 * 
 * Feel free to ignore this file
 * ==================================
 */
public class MotionConfig {
    private PID pid;
    private MotionMagic mm;
    private ControlType controlType;

    private int uses = 0;

    // nomenclature 100
    public MotionConfig pid(PID pid) {
        this.pid = pid;

        return this;
    }

    public MotionConfig motionMagic(MotionMagic mm) {
        this.mm = mm;

        return this;
    }

    public MotionConfig controller(ControlType controlType) {
        this.controlType = controlType;

        return this;
    }


    
    public PID getPID() {
        return pid;
    }

    public MotionMagic getMotionMagic() {
        return mm;
    }

    public ControlType getControlType() {
        return controlType;
    }





    public void use() {
        uses++;
    }

    public int getUses() {
        return uses;
    }
}