package frc.robot.utils.data.noise;

import org.ejml.simple.SimpleMatrix;

/*
 * ==================================
 * 2020 Code Review Party Information
 * 
 * Feel free to ignore this file
 * ==================================
 */
public abstract class NoiseSource {
    protected final int NUM_STATES;
    protected final int NUM_NOISES;

    public NoiseSource(int numStates, int numNoises) {
        NUM_STATES = numStates;
        NUM_NOISES = numNoises;
    }



    public abstract Noise getNoise(SimpleMatrix x, SimpleMatrix u, double t, int k);





    public static NoiseSource def(int numStates) {
        return new ConstantNoiseSource(numStates, numStates, new SimpleMatrix(numStates, numStates), new SimpleMatrix(numStates, numStates));
    };
}