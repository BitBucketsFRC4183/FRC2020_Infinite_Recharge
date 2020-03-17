package frc.robot.utils.control.statespace.models.ltif;

import org.ejml.simple.SimpleMatrix;

import frc.robot.utils.control.statespace.models.ABFTriple;
import frc.robot.utils.control.statespace.models.C2D;



/*
 * ==================================
 * 2020 Code Review Party Information
 * 
 * Feel free to ignore this file
 * ==================================
 */
public class CLTIFModel extends LTIFModel {
    protected final double DT;

    public CLTIFModel(SimpleMatrix A, SimpleMatrix B, SimpleMatrix F, double dt) {
        super(C2D.c2d(A, B, F, dt));

        DT = dt;
    }

    public CLTIFModel(ABFTriple ABF, double dt) {
        super(C2D.c2d(ABF, dt));

        DT = dt;
    }
}