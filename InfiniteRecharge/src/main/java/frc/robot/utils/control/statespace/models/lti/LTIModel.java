package frc.robot.utils.control.statespace.models.lti;

import org.ejml.simple.SimpleMatrix;

import frc.robot.utils.control.statespace.models.ABCouple;
import frc.robot.utils.control.statespace.models.ABFTriple;
import frc.robot.utils.control.statespace.models.ltif.LTIFModel;



public class LTIModel extends LTIFModel {
    public LTIModel(ABCouple AB) {
        this(AB.getA(), AB.getB());
    }

    public LTIModel(SimpleMatrix A, SimpleMatrix B) {
        super(new ABFTriple(A, B, new SimpleMatrix(A.numRows(), A.numRows())));
    }
}