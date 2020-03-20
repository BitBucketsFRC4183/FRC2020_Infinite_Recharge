package frc.robot.utils.control.statespace.models.linearized;

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
public abstract class CLinearizedModel extends LinearizedModel {
    protected final double DT;



    public CLinearizedModel(int states, int inputs, double dt) {
        super(states, inputs);

        this.DT = dt;
    }



    @Override
    protected SimpleMatrix updateA(SimpleMatrix stateVector, double t, int k) { return null; }

    @Override
    protected SimpleMatrix updateB(SimpleMatrix stateVector, double t, int k) { return null; }

    @Override
    protected SimpleMatrix updateF(SimpleMatrix stateVector, double t, int k) { return null; }
    


    public abstract SimpleMatrix updateAc(SimpleMatrix stateVector, double t, int k);
    public abstract SimpleMatrix updateBc(SimpleMatrix stateVector, double t, int k);
    public abstract SimpleMatrix updateFc(SimpleMatrix stateVector, double t, int k);



    @Override
    public ABFTriple getDiscreteSystem(SimpleMatrix stateVector, double t, int k) {
        SimpleMatrix Ac = updateAc(stateVector, t, k);
        SimpleMatrix Bc = updateBc(stateVector, t, k);
        SimpleMatrix Fc = updateFc(stateVector, t, k);

        return C2D.c2d(Ac, Bc, Fc, DT);
    }
}