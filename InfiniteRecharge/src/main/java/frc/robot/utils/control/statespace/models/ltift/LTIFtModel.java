package frc.robot.utils.control.statespace.models.ltift;

import frc.robot.utils.control.statespace.models.ABCouple;
import frc.robot.utils.control.statespace.models.linearized.LinearizedModel;

import org.ejml.simple.SimpleMatrix;



/*
 * ==================================
 * 2020 Code Review Party Information
 * 
 * Feel free to ignore this file
 * ==================================
 */
/** LTI model + F(t), without any dependence of control u on output y */
public abstract class LTIFtModel extends LinearizedModel {
    // LTI
    protected final ABCouple AB;



    public LTIFtModel(ABCouple AB) {
        super(AB.getA().numRows(), AB.getB().numCols());

        this.AB = new ABCouple(AB.getA(), AB.getB());
    }

    public LTIFtModel(SimpleMatrix A, SimpleMatrix B) {
        this(new ABCouple(A, B));
    }



    @Override
    protected SimpleMatrix updateA(SimpleMatrix stateVector, double t, int k) {
        return AB.getA();
    }

    @Override
    protected SimpleMatrix updateB(SimpleMatrix stateVector, double t, int k) {
        return AB.getB();
    }

    // F does have to be provided, but is only time varying
    @Override
    protected SimpleMatrix updateF(SimpleMatrix stateVector, double t, int k) {
        return updateF(t, k);
    }

    protected abstract SimpleMatrix updateF(double t, int k);



    public ABCouple getABCouple() {
        return AB;
    }

    public SimpleMatrix getA() {
        return AB.getA();
    }

    public SimpleMatrix getB() {
        return AB.getB();
    }
}