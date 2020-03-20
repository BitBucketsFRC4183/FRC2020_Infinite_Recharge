package frc.robot.utils.control.statespace.models.linearized;

import frc.robot.utils.control.statespace.models.StateSpaceModel;
import frc.robot.utils.control.statespace.models.ABFTriple;

import java.io.IOException;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpilibj.Timer;



/*
 * ==================================
 * 2020 Code Review Party Information
 * 
 * Feel free to ignore this file
 * ==================================
 */
/**
 * Linearized model of a system's dynamics.
 * The real system can be given by:
 *     x_{k + 1} = f(x_k, u_k, t)
 * The linearized system is given by:
 *     x_{k + 1} = A(x_k, u_k, t) x_k + B(x_k, u_t, t) + F(x_k, u_t, t)
 * for some matrices A, B, and F.
 * 
 * However, If A, B, and F depend on u, it will be harder to determine
 * the appropritate control to apply given the system model, so we'll
 * assume
 *     x_{k + 1} = A(x_k, t) x_k + B(x_k, t) + F(x_k, t)
 * 
 * We can design a controller to produce a matrix K such that
 *     x_{k + 1} approx (A-BK)x_k
 * by applying u = -Kx_k - B'F
 * where B' is the Moore-Penrose pseudoinverse of B
 */
public abstract class LinearizedModel extends StateSpaceModel {
    protected ABFTriple lastSystem = null;

    //protected ABFTriple ABF;



    /**
     * Create a linearized model of a system
     * 
     * @param states number of states in the system
     * @param inputs number of inputs to the system
     */
    public LinearizedModel(int states, int inputs) {
        super(states, inputs);
    }



    /**
     * ONLY TO BE CALLED BY getDiscreteSystem()
     * Update the matrix A given a state, input, and time.
     * 
     * @param stateVector
     * @param inputVector
     * @param t
     * @return
     */
    protected abstract SimpleMatrix updateA(SimpleMatrix stateVector, double t, int k);
    protected abstract SimpleMatrix updateB(SimpleMatrix stateVector, double t, int k);
    protected abstract SimpleMatrix updateF(SimpleMatrix stateVector, double t, int k);



    public ABFTriple getDiscreteSystem(SimpleMatrix stateVector, double t, int k) {
        return new ABFTriple(
            updateA(stateVector, t, k),
            updateB(stateVector, t, k),
            updateF(stateVector, t, k)
        );
    }

    protected ABFTriple getDiscreteSystem() {
        lastSystem = getDiscreteSystem(state, lastTime, k);

        return lastSystem;
    }

    @Override
    public SimpleMatrix update(SimpleMatrix stateVector, SimpleMatrix inputVector, double t, int k) {
        ABFTriple abf = getDiscreteSystem(stateVector, t, k);

        return (abf.getA().mult(stateVector))
            .plus(abf.getB().mult(inputVector))
            .plus(abf.getF());
    }

    @Override
    public SimpleMatrix update() {
        return (lastSystem.getA().mult(state))
            .plus(lastSystem.getB().mult(input))
            .plus(lastSystem.getF());
    }

    @Override
    public void apply(SimpleMatrix inputVector) {
        getDiscreteSystem();

        super.apply(inputVector);
    }



    public ABFTriple getLastSystem() { return lastSystem; }
}