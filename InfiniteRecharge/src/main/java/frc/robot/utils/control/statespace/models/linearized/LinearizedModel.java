package frc.robot.utils.control.statespace.models.linearized;



import frc.robot.utils.control.statespace.models.StateSpaceModel;
import frc.robot.utils.control.statespace.models.ABFTriple;

import org.ejml.simple.SimpleMatrix;



/**
 * Linearized model of a system's dynamics.
 * The real system can be given by:
 *     x_{k + 1} = f(x_k, u_k, t)
 * The linearized system is given by:
 *     x_{k + 1} = A(x_k, u_k, t) x_k + B(x_k, u_t, t) + F(x_k, u_t, t)
 * for some matrices A, B, and F
 */
public abstract class LinearizedModel extends StateSpaceModel {
    protected final int NUM_STATES;
    protected final int NUM_INPUTS;

    protected SimpleMatrix state;
    protected SimpleMatrix input;

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
    protected abstract SimpleMatrix updateA(SimpleMatrix stateVector, SimpleMatrix inputVector, double t);
    protected abstract SimpleMatrix updateB(SimpleMatrix stateVector, SimpleMatrix inputVector, double t);
    protected abstract SimpleMatrix updateF(SimpleMatrix stateVector, SimpleMatrix inputVector, double t);



    public ABFTriple getDiscreteSystem(SimpleMatrix stateVector, SimpleMatrix inputVector, double t) {
        return new ABFTriple(
            updateA(stateVector, inputVector, t),
            updateB(stateVector, inputVector, t),
            updateF(stateVector, inputVector, t)
        );
    }

    public SimpleMatrix update(SimpleMatrix stateVector, SimpleMatrix inputVector, double t) {
        ABFTriple abf = getDiscreteSystem(stateVector, inputVector, t);

        return (abf.getA().mult(stateVector))
            .plus(abf.getB().mult(inputVector))
            .plus(abf.getF());
    }

    public SimpleMatrix update(double t) {
        state = update(state, input, t);

        return state;
    }

    public void apply(SimpleMatrix inputVector) {
        input = inputVector;
    }
}