package frc.robot.utils.control.statespace.models.linearized;

import frc.robot.utils.control.statespace.models.StateSpaceModel;
import frc.robot.utils.control.statespace.models.ABFTriple;

import java.io.IOException;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpilibj.Timer;



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
    protected final int NUM_STATES;
    protected final int NUM_INPUTS;

    protected SimpleMatrix state;
    protected SimpleMatrix input;

    protected double lastTime;
    protected double t0;
    protected int k;

    protected ABFTriple lastSystem = null;

    //protected ABFTriple ABF;



    /**
     * Create a linearized model of a system
     * 
     * @param states number of states in the system
     * @param inputs number of inputs to the system
     */
    public LinearizedModel(int states, int inputs) {
        NUM_STATES = states;
        NUM_INPUTS = inputs;

        state = new SimpleMatrix(NUM_STATES, 1);
        input = new SimpleMatrix(NUM_INPUTS, 1);

        // default to this so methods have something
        t0 = getTotalTime();
        k = 0;
    }



    public void resetTimer() {
        t0 = getTotalTime();
        lastTime = 0;
        k = 0;
    }

    public int getCount() {
        return k;
    }

    private double getTotalTime() {
        return System.nanoTime() * 1000000;
        //return System.currentTimeMillis() / 1000.0;
        //return Timer.getFPGATimestamp();
        /*try {
            return Timer.getFPGATimestamp();
        } finally {
            return System.currentTimeMillis() / 1000;
        }*/
    }

    public double getTime() {
        return getTotalTime() - t0;
    }

    public double getLastTime() {
        return lastTime;
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
        lastTime = getTime();
        lastSystem = getDiscreteSystem(state, lastTime, k);

        return lastSystem;
    }

    public SimpleMatrix update(SimpleMatrix stateVector, SimpleMatrix inputVector, double t, int k) {
        ABFTriple abf = getDiscreteSystem(stateVector, t, k);

        return (abf.getA().mult(stateVector))
            .plus(abf.getB().mult(inputVector))
            .plus(abf.getF());
    }

    public SimpleMatrix update() {
        return (lastSystem.getA().mult(state))
            .plus(lastSystem.getB().mult(input))
            .plus(lastSystem.getF());
    }

    public void apply(SimpleMatrix inputVector) {
        getDiscreteSystem();

        input = inputVector;

        state = update();
        
        k++;
    }



    public SimpleMatrix getState() { return state; }
    public SimpleMatrix getInput() { return input; }
    public ABFTriple getLastSystem() { return lastSystem; }

    public void setState(SimpleMatrix state) { this.state = state; }

    public int getNumInputs() { return NUM_INPUTS; }
    public int getNumStates() { return NUM_STATES; }
}