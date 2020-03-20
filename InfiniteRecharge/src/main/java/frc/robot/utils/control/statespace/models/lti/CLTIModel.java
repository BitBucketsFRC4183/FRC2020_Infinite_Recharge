package frc.robot.utils.control.statespace.models.lti;

import org.ejml.simple.SimpleMatrix;

import frc.robot.utils.control.statespace.models.ABCouple;
import frc.robot.utils.control.statespace.models.C2D;

/*
 * ==================================
 * 2020 Code Review Party Information
 * 
 * Feel free to ignore this file
 * ==================================
 */
public abstract class CLTIModel extends LTIModel {
    protected final double DT;

    public CLTIModel(ABCouple AB, double dt) {
        super(C2D.c2d(AB, dt));
        
        DT = dt;
    }

    public CLTIModel(SimpleMatrix A, SimpleMatrix B, double dt) {
        super(C2D.c2d(A, B, dt));
        
        DT = dt;
    }
}