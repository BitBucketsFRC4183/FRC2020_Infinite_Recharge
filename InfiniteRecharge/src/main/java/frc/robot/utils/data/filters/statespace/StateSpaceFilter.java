package frc.robot.utils.data.filters.statespace;

import org.ejml.simple.SimpleMatrix;

import frc.robot.utils.data.filters.Filter;

public abstract class StateSpaceFilter extends Filter<SimpleMatrix> {
    public abstract void postApply();
}