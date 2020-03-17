package frc.robot.utils.control.statespace.observer;

import org.ejml.simple.SimpleMatrix;

import frc.robot.utils.data.noise.NoiseSource;


/*
 * ==================================
 * 2020 Code Review Party Information
 * 
 * Feel free to ignore this file
 * ==================================
 */
// literally just a sensor lol
public abstract class OutputObserver {
    protected final int NUM_OUTPUTS;
    protected NoiseSource noise;

    public OutputObserver(int numOutputs) {
        NUM_OUTPUTS = numOutputs;

        noise = NoiseSource.def(numOutputs);
    }

    public void addNoiseSource(NoiseSource noise) {
        this.noise = noise;
    }

    public int getNumOutputs() { return NUM_OUTPUTS; }

    public NoiseSource getNoiseSource() {
        return noise;
    }

    public abstract SimpleMatrix getOutput(SimpleMatrix state, SimpleMatrix input, double t, int k);
}