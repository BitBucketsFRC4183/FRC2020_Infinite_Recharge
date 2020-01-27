package frc.robot.utils.control.statespace.models.ltif;

import org.ejml.simple.SimpleMatrix;

import frc.robot.utils.control.statespace.models.ABFTriple;
import frc.robot.utils.control.statespace.models.ltift.LTIFtModel;



public class LTIFModel extends LTIFtModel {
    protected final SimpleMatrix F;

    public LTIFModel(ABFTriple ABF) {
        super(ABF.getCouple());

        F = ABF.getF();
    }

    @Override
    protected SimpleMatrix updateF(double t, int k) {
        return F;
    }
}