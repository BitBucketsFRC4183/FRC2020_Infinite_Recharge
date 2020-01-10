/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

public enum JoystickScale {
    LINEAR,
    SQUARE,
    CUBE,
    SINE;



    // Some utility functions to make later code more readable
    // All of these function preserve sign of input

    /**
     * rescale with a deadband
     * 
     */
    public static double map(double x, double inMin, double inMax, double outMin, double outMax) {
        return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }


    
    /**
     * Rescale without a deadband
     * 
     */
    public double rescale(double x) {
        switch (this) {
            case SQUARE: // Decrease sensitivity
                x = square(x);
                break;
            case CUBE: // Decrease sensitivity a little more
                x = cube(x);
                break;
            case SINE: // Slightly decreased sensitivity and then approach linear
                x = sine(x);
                break;
            case LINEAR:
            default:
                break;
        }

        return x;
    }

    public double rescale(double x, double deadband) {
        // Apply deadband first, remaping slope as needed to prevent jackrabbit changes
        deadband = Math.abs(deadband);
        if (Math.abs(x) >= deadband) {
            // Above deadband.
            // Re-zero at deadband
            if (x < 0) {
                x = map(x, -1.0, -deadband, -1.0, 0);
            } else {
                x = map(x, deadband, 1.0, 0, 1.0);
            }
        } else {
            x = 0;
        }

        return rescale(x);
    }



    public double square(double x) {
        return Math.abs(x) * x;
    }

    public double cube(double x) {
        return x * x * x;
    }

    public double sine(double x) {
        x = Math.abs(x) * Math.sin(x * Math.PI / 2.0);

        return x;
    }
}