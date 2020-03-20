package frc.robot.utils.control.statespace.models.ltift;

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
public abstract class CLTIFtModel extends LTIFtModel {
    private final double DT;



    protected final SimpleMatrix GAMMA;



    // Java bad
    private CLTIFtModel(SimpleMatrix A, SimpleMatrix B, double dt, SimpleMatrix S, SimpleMatrix gamma) {
        super(
            C2D.getA(S, A.numRows()),
            gamma.mult(B)
        );

        GAMMA = gamma;

        DT = dt;
    }

    private CLTIFtModel(SimpleMatrix A, SimpleMatrix B, double dt, SimpleMatrix S) {
        this(A, B, dt, S, C2D.getGamma(S, A.numRows()));
    }

    public CLTIFtModel(ABCouple AB, double dt) {
        this(AB.getA(), AB.getB(), dt);
    }

    public CLTIFtModel(SimpleMatrix A, SimpleMatrix B, double dt) {
        this(A, B, dt, C2D.getS(A, dt));
    }


    
    @Override
    protected SimpleMatrix updateF(double t, int k) {
        return GAMMA.mult(updateFc(t));
    }

    public abstract SimpleMatrix updateFc(double t);
}