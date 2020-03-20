package frc.robot.utils.control.statespace;

import org.ejml.data.DMatrix1Row;

/*
 * ==================================
 * 2020 Code Review Party Information
 * 
 * Feel free to ignore this file
 * ==================================
 */
public abstract class Reference {
    public abstract DMatrix1Row getReference();
}