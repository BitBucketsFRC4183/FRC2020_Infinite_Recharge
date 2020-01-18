package frc.robot.utils.control.statespace.models.lti;

import frc.robot.utils.control.statespace.StateSpaceException;

import org.ejml.simple.SimpleMatrix;



public abstract class LTIModel extends LTIKModel {
    public LTIModel(SimpleMatrix A, SimpleMatrix B, SimpleMatrix C) throws StateSpaceException {
        super(A, B, C, (new SimpleMatrix(A.numRows(), 1)));
    }

    public LTIModel(SimpleMatrix A, SimpleMatrix B) throws StateSpaceException {
        super(A, B, null, (new SimpleMatrix(A.numRows(), 1)));
    }

    public LTIModel(SimpleMatrix[] mats) throws StateSpaceException {
        // fun fact: I hate Java
        this(mats[0], mats[1], (mats.length >= 3) ? mats[2] : null);
    }
}