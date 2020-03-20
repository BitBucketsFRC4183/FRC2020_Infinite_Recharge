package frc.robot.utils.control.statespace.observer;

import org.ejml.simple.SimpleMatrix;

/*
 * ==================================
 * 2020 Code Review Party Information
 * 
 * Feel free to ignore this file
 * ==================================
 */
public abstract class LinearizedOutputObserver extends OutputObserver {
    public LinearizedOutputObserver(int numOutputs) {
        super(numOutputs);
    }

    public abstract SimpleMatrix getJacobian(SimpleMatrix state, SimpleMatrix input, double t, int k);
}