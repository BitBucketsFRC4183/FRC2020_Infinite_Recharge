package frc.robot.utils.control.statespace.models.ltift;

import frc.robot.utils.control.statespace.models.ABCouple;
import frc.robot.utils.control.statespace.models.linearized.LinearizedModel;

import org.ejml.simple.SimpleMatrix;



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
    protected SimpleMatrix updateA(SimpleMatrix stateVector, SimpleMatrix inputVector, double t) {
        return AB.getA();
    }

    @Override
    protected SimpleMatrix updateB(SimpleMatrix stateVector, SimpleMatrix inputVector, double t) {
        return AB.getB();
    }

    // F does have to be provided, but is only time varying
    @Override
    protected SimpleMatrix updateF(SimpleMatrix stateVector, SimpleMatrix inputVector, double t) {
        return updateF(t);
    }

    protected abstract SimpleMatrix updateF(double t);
}