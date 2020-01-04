package frc.robot.utils.control.statespace;

import org.ejml.data.DMatrix1Row;

public abstract class Reference {
    public abstract DMatrix1Row getReference();
}