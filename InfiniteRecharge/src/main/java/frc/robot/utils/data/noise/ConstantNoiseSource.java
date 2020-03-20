package frc.robot.utils.data.noise;

import org.ejml.simple.SimpleMatrix;

/*
 * ==================================
 * 2020 Code Review Party Information
 * 
 * Feel free to ignore this file
 * ==================================
 */
public class ConstantNoiseSource extends NoiseSource {
    private Noise noise;

    public ConstantNoiseSource(int numStates, SimpleMatrix Q) {
        this(numStates, numStates, SimpleMatrix.identity(numStates), Q);
    }

    public ConstantNoiseSource(int numStates, int numNoises, SimpleMatrix G, SimpleMatrix Q) {
        super(numStates, numNoises);

        noise = new Noise(G, Q);
    }


    
    @Override
    public Noise getNoise(SimpleMatrix x, SimpleMatrix u, double t, int k) {
        return noise;
    }
}