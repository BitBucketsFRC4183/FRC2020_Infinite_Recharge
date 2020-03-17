package frc.robot.utils.control.statespace.models;

import org.apache.commons.math3.exception.DimensionMismatchException;
import org.apache.commons.math3.exception.MaxCountExceededException;
import org.apache.commons.math3.ode.FirstOrderDifferentialEquations;
import org.apache.commons.math3.ode.nonstiff.DormandPrince853Integrator;

import org.ejml.simple.SimpleMatrix;

/*
 * ==================================
 * 2020 Code Review Party Information
 * 
 * Feel free to ignore this file
 * ==================================
 */
public abstract class CStateSpaceModel extends StateSpaceModel {
    private double[] inputVectorTemp;
    private double[] stateVectorTemp;

    private final FirstOrderDifferentialEquations ode;
    private final DormandPrince853Integrator integrator;

    private final double dt;
    
    public CStateSpaceModel(int numStates, int numInputs, double dt, double minStep, double maxStep, double[] absTol, double[] relTol) {
        super(numStates, numInputs);

        this.dt = dt;



        inputVectorTemp = new double[NUM_INPUTS];
        stateVectorTemp = new double[NUM_STATES];

        ode = new FirstOrderDifferentialEquations() {
			@Override
			public int getDimension() {
				return NUM_STATES;
			}

			@Override
			public void computeDerivatives(double t, double[] y, double[] yDot) throws MaxCountExceededException, DimensionMismatchException {
                deriv(y, inputVectorTemp, t, getCount(), yDot);
			}
        };

        // called every 0.02s, so go lower
        integrator = new DormandPrince853Integrator(minStep, maxStep, absTol, relTol);
    }



    public abstract void deriv(double[] x, double[] u, double t, int k, double[] xDot);

    @Override
    public SimpleMatrix update(SimpleMatrix stateVector, SimpleMatrix inputVector, double t, int k) {
        for (int i = 0; i < NUM_INPUTS; i++) {
            inputVectorTemp[i] = inputVector.get(i); // matrix stored internally as an array but doesn't let me access it >:(
        }

        for (int i = 0; i < NUM_STATES; i++) {
            stateVectorTemp[i] = stateVector.get(i); // matrix stored internally as an array but doesn't let me access it >:(
        }

        integrator.integrate(ode, getLastTime(), stateVectorTemp, getLastTime() + dt, stateVectorTemp);

        return new SimpleMatrix(NUM_STATES, 1, true, stateVectorTemp);
    }
    
}