package frc.robot.utils.control.statespace.modeling.ildata;

import frc.robot.utils.control.motor.BBMotorController;
import frc.robot.utils.roborio.RoboRIOFS;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;

import java.io.IOException;
import java.io.File;
import java.io.FileWriter;
import java.io.PrintWriter;



/*
 * ==================================
 * 2020 Code Review Party Information
 * 
 * Feel free to ignore this file
 * ==================================
 */
/**
 * Measure moment of inertia (I) and inductance (L) through a motor by running it
 * at a set voltage. Warning: this requires changing all motor units to SI and
 * having no gear ratios or load.
 */
public class ILData {
    private static final double PERIOD_SEC = 0.01; // periodt

    private static final int MEASUREMENTS_PER_TRIAL = 100;
    private static final int TRIALS = 10;



    public static class DataPoint {
        private final double TIME0;
        private final double THETA;
        private final double OMEGA;
        private final double CURRENT;
        private final double CURRENT_DERIV;
        private final double VOLTAGE;

        private DataPoint(double time0, double theta, double omega, double current, double currentDeriv, double voltage) {
            TIME0 = time0;
            THETA = theta;
            OMEGA = omega;
            CURRENT = current;
            CURRENT_DERIV = currentDeriv;
            VOLTAGE = voltage;
        }

        public String toString() {
            return TIME0 + " " + THETA + " " + OMEGA + " " + CURRENT + " " + CURRENT_DERIV + " " + VOLTAGE;
        }
    }



    private static enum State {
        Measurement, // measurement state: constant voltage is applied. if voltage deviates from a
                     // constant value, move back to REST state and try again to get more accurate data
        Rest; // resting state: no voltage applied, slow down motors to 0 current and angular velocity
    }



    

    private final BBMotorController MOTOR;
    private final double PERCENT;

    private int trialNum = 0;

    private Notifier notifier;
    private State state = State.Rest;



    // measurement stuff
    private double time0;
    private double lastCurrent = 0;
    private DataPoint[] dataPoints = new DataPoint[MEASUREMENTS_PER_TRIAL];
    private int measurementNum = 0;
    

    
    public ILData(BBMotorController motor, double percent) {
        MOTOR = motor;
        PERCENT = percent;



        MOTOR.setSI();
        MOTOR.setInverted(false);
        MOTOR.setSensorPhase(false);
        MOTOR.setOpenLoopRampRate(0);
        MOTOR.zero();



        File motorFile = new File(RoboRIOFS.MOTOR_DATA_NAME + MOTOR.getDeviceID() + "/");
        if (!motorFile.exists()) {
            motorFile.mkdir();
        }
    }



    public void run() {
        notifier = new Notifier(new Loop());
        notifier.startPeriodic(PERIOD_SEC); // 5 ms loop
    }



    private void writeMeasurements() {
        String pathName = RoboRIOFS.MOTOR_DATA_NAME + MOTOR.getDeviceID() + "/trial" + trialNum + ".txt";

        File file = new File(pathName);

        if (!file.exists()) {
            try {
                file.createNewFile();
            } catch (IOException e) {
                System.out.println("Couldn'tcreatefile");
            }
        }

        try {
            FileWriter fileWriter = new FileWriter(pathName);
            PrintWriter writer = new PrintWriter(fileWriter);

            for (int i = 0; i < MEASUREMENTS_PER_TRIAL; i++) {
                writer.println(dataPoints[i].toString());
            }

            writer.close();
        } catch (IOException e) {
            System.out.println("Error: " + e);
        }
    }



    private class Loop implements Runnable {
        @Override
        public void run() {
            if (state == State.Measurement) {
                MOTOR.cmdPercent(PERCENT);

                double voltage = MOTOR.getVoltage();

                double theta = MOTOR.getPosition();
                double omega = MOTOR.getVelocity();
                double current = MOTOR.getCurrent();
                double currentDeriv = (current - lastCurrent) / PERIOD_SEC;

                lastCurrent = current; // set the last current to the current current

                double time = Timer.getFPGATimestamp() - time0;

                DataPoint dp = new DataPoint(time, theta, omega, current, currentDeriv, voltage);
                dataPoints[measurementNum] = dp;

                measurementNum++;
                if (measurementNum == MEASUREMENTS_PER_TRIAL) {
                    writeMeasurements();

                    trialNum++;

                    MOTOR.cmdPercent(0); // rest the motor
                    state = State.Rest;

                    lastCurrent = 0;
                    measurementNum = 0;
                }
            } else if (state == State.Rest) {
                double omega = MOTOR.getVelocity();
                double current = MOTOR.getCurrent();
                double voltage = MOTOR.getVoltage();

                System.out.println("Voltage: " + voltage);

                if (omega == 0 && voltage == 0) {
                    if (trialNum == TRIALS) {
                        notifier.stop();
                    } else {
                        time0 = Timer.getFPGATimestamp();

                        MOTOR.zero();
                        MOTOR.cmdPercent(PERCENT);

                        state = State.Measurement;
                    }
                }
            }
        }
    }
}