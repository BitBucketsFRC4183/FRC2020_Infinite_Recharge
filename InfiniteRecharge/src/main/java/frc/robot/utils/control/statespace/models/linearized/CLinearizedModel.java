package frc.robot.utils.control.statespace.models.linearized;

import org.ejml.simple.SimpleMatrix;

import frc.robot.utils.control.statespace.models.ABFTriple;
import frc.robot.utils.control.statespace.models.C2D;



public abstract class CLinearizedModel extends LinearizedModel {
    protected final double DT;



    public CLinearizedModel(int states, int inputs, double dt) {
        super(states, inputs);

        this.DT = dt;
    }



    @Override
    protected SimpleMatrix updateA(SimpleMatrix stateVector, SimpleMatrix inputVector, double t) { return null; }

    @Override
    protected SimpleMatrix updateB(SimpleMatrix stateVector, SimpleMatrix inputVector, double t) { return null; }

    @Override
    protected SimpleMatrix updateF(SimpleMatrix stateVector, SimpleMatrix inputVector, double t) { return null; }
    


    public abstract SimpleMatrix updateAc(SimpleMatrix stateVector, SimpleMatrix inputVector, double t);
    public abstract SimpleMatrix updateBc(SimpleMatrix stateVector, SimpleMatrix inputVector, double t);
    public abstract SimpleMatrix updateFc(SimpleMatrix stateVector, SimpleMatrix inputVector, double t);



    @Override
    public ABFTriple getDiscreteSystem(SimpleMatrix stateVector, SimpleMatrix inputVector, double t) {
        SimpleMatrix Ac = updateAc(stateVector, inputVector, t);
        SimpleMatrix Bc = updateBc(stateVector, inputVector, t);
        SimpleMatrix Fc = updateFc(stateVector, inputVector, t);

        return C2D.c2d(Ac, Bc, Fc, DT);
    }
}