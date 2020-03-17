package frc.robot.utils.control.statespace.models;

import org.ejml.simple.SimpleMatrix;

import frc.robot.utils.data.noise.NoiseSource;


/*
 * ==================================
 * 2020 Code Review Party Information
 * 
 * Feel free to ignore this file
 * ==================================
 */
// DISCRETE
public abstract class StateSpaceModel {
    protected final int NUM_STATES;
    protected final int NUM_INPUTS;

    protected SimpleMatrix state;
    protected SimpleMatrix input;

    protected double lastTime;
    protected double t0;
    protected int k;

    protected NoiseSource noise;



    public StateSpaceModel(int numStates, int numInputs) {
        NUM_STATES = numStates;
        NUM_INPUTS = numInputs;

        state = new SimpleMatrix(NUM_STATES, 1);
        input = new SimpleMatrix(NUM_INPUTS, 1);

        // default to this so methods have something
        t0 = getTotalTime();
        k = 0;

        noise = NoiseSource.def(numStates);
    }



    public void addNoiseSource(NoiseSource noise) {
        this.noise = noise;
    }

    public NoiseSource getNoiseSource() { return noise; }



    public void resetTimer() {
        t0 = getTotalTime();
        lastTime = 0;
        k = 0;
    }

    public int getCount() {
        return k;
    }

    protected double getTotalTime() {
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



    public abstract SimpleMatrix update(SimpleMatrix stateVector, SimpleMatrix inputVector, double t, int k);
    public SimpleMatrix update() {
        return update(state, input, lastTime, k);
    }
    public SimpleMatrix propogate(SimpleMatrix stateVector) {
        return update(stateVector, input, lastTime, k);
    };



    public void apply(SimpleMatrix inputVector) {
        lastTime = getTime();

        input = inputVector;
        
        k++;
    }



    public SimpleMatrix getState() { return state; }
    public SimpleMatrix getInput() { return input; }



    public void setState(SimpleMatrix state) { this.state = state; }

    public int getNumInputs() { return NUM_INPUTS; }
    public int getNumStates() { return NUM_STATES; }
}