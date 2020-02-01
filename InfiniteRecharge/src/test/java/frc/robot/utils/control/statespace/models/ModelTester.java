package frc.robot.utils.control.statespace.models;

import static org.junit.Assert.assertTrue;

import org.ejml.data.DMatrixRMaj;
import org.ejml.simple.SimpleMatrix;
import org.junit.Test;

import frc.robot.subsystem.navigation.RobotSystem;
import frc.robot.utils.control.statespace.models.motor.MotorPosition;
import frc.robot.utils.control.statespace.models.motor.MotorVelocity;
import frc.robot.utils.control.statespace.models.motors.MotorType;
import frc.robot.utils.data.DoubleDataWindow;

public class ModelTester {
    private static final double TOLERANCE = 0.000001;

    @Test
    public void c2d() {
        // state = [x, v];
        double m = 2;
        // input = F
        // F = friction
        double Ff = -0.5;

        double dt = 0.2;

        SimpleMatrix A = new SimpleMatrix(new double[][] { new double[] { 0, 1 }, new double[] { 0, 0 } });

        SimpleMatrix B = new SimpleMatrix(new double[][] { new double[] { 0 }, new double[] { 1 / m } });

        SimpleMatrix F = new SimpleMatrix(new double[][] { new double[] { 0 }, new double[] { Ff / m } });

        SimpleMatrix Ad = new SimpleMatrix(new double[][] { new double[] { 1, dt }, new double[] { 0, 1 } });

        SimpleMatrix Bd = new SimpleMatrix(
                new double[][] { new double[] { 1 / (2 * m) * dt * dt }, new double[] { 1 / m * dt } });

        SimpleMatrix Fd = new SimpleMatrix(
                new double[][] { new double[] { 1 / (2 * m) * dt * dt * Ff }, new double[] { 1 / m * dt * Ff } });

        ABFTriple abf = new ABFTriple(A, B, F);
        double t0 = System.currentTimeMillis();
        abf = C2D.c2d(abf, dt);
        double t1 = System.currentTimeMillis();

        System.out.println("Discretized in " + (t1 - t0) + "ms");
        System.out.println(abf.getA().isIdentical(Ad, 0.001));

        System.out.println("A matrix:");
        System.out.println(abf.getA().toString());
        System.out.println();

        System.out.println("B matrix:");
        System.out.println(abf.getB().toString());
        System.out.println();

        System.out.println("F matrix:");
        System.out.println(abf.getF().toString());
        System.out.println();

        assertTrue("A wasn't discretized correctly", abf.getA().isIdentical(Ad, TOLERANCE));
        assertTrue("B wasn't discretized correctly", abf.getB().isIdentical(Bd, TOLERANCE));
        assertTrue("F wasn't discretized correctly", abf.getF().isIdentical(Fd, TOLERANCE));
    }

    @Test
    public void motorPosition() {
        double t0 = System.nanoTime();

        MotorType type = MotorType.Falcon500;
        double I = 2;

        double b = type.getB().getValue();
        double Kw = type.getKW().getValue();
        double Kt = type.getKT().getValue();
        double R = type.getR().getValue();

        double A = -(b / I + Kw * Kt / (R * I));
        double B = Kt / (I * R);

        double dt = 0.02;
        double t1 = System.nanoTime();

        /*
         * SimpleMatrix Ac = new SimpleMatrix(new double[][] { new double[] {0, 1}, new
         * double[] {0, A} });
         * 
         * SimpleMatrix Bc = new SimpleMatrix(new double[][] { new double[] {0}, new
         * double[] {B} });
         */

        DMatrixRMaj Ad, Bd;

        double t2 = System.nanoTime();

        Ad = new DMatrixRMaj(2, 2, true, new double[] { 1, (Math.exp(dt * A) - 1) / A, 0, Math.exp(dt * A) });
        // new double[][] {
        // new double[] {1, (Math.exp(dt * A) - 1) / A},
        // new double[] {0, Math.exp(dt * A)}
        // });

        Bd = new DMatrixRMaj(2, 1, true,
                new double[] { ((Math.exp(A * dt) - 1) / (A * A) - dt / A) * B, (Math.exp(A * dt) - 1) / A * B });
        // });

        double t3 = System.nanoTime();

        System.out.println("Setup time: " + (t1 - t0) / 1000000.0 + "ms");
        System.out.println("Continuous system creation time: " + (t2 - t1) / 1000000.0 + "ms");
        System.out.println("Discrete system creation time: " + (t3 - t2) / 1000000.0 + "ms");
        System.out.println("Total time: " + (t3 - t0) / 1000000.0 + "ms");

        MotorPosition model = new MotorPosition(dt, type, I);

        // assertTrue(Ac.isIdentical(MotorPosition.getA(type, I), TOLERANCE));
        // assertTrue(Bc.isIdentical(MotorPosition.getB(type, I), TOLERANCE));
        // assertTrue(Ad.isIdentical(model.getA(), TOLERANCE));
        // assertTrue(Bd.isIdentical(model.getB(), TOLERANCE));
    }

    @Test
    public void motorVelocity() {
        MotorType type = MotorType.Falcon500;
        double I = 2;

        double b = type.getB().getValue();
        double Kw = type.getKW().getValue();
        double Kt = type.getKT().getValue();
        double R = type.getR().getValue();

        double A = -(b / I + Kw * Kt / (R * I));
        double B = Kt / (I * R);

        double dt = 0.02;

        MotorVelocity model = new MotorVelocity(dt, type, I);

        SimpleMatrix Ac = new SimpleMatrix(new double[][] { new double[] { A } });
        assertTrue(Ac.isIdentical(MotorVelocity.getA(type, I), TOLERANCE));

        SimpleMatrix Bc = new SimpleMatrix(new double[][] { new double[] { B } });
        assertTrue(Bc.isIdentical(MotorVelocity.getB(type, I), TOLERANCE));

        SimpleMatrix Ad = new SimpleMatrix(new double[][] { new double[] { Math.exp(dt * A) } });

        SimpleMatrix Bd = new SimpleMatrix(new double[][] { new double[] { (Math.exp(A * dt) - 1) / A * B } });

        assertTrue(Ad.isIdentical(model.getA(), TOLERANCE));
        assertTrue(Bd.isIdentical(model.getB(), TOLERANCE));
    }

    @Test
    public void robotSystem() {
        RobotSystem sys = new RobotSystem();

        DoubleDataWindow window = new DoubleDataWindow(50);

        for (int i = 0; i < 51; i++) {
            double t0 = System.nanoTime();
            sys.getModel(0.01, 0.02);
            double t1 = System.nanoTime();
            double dt = (t1 - t0) / 1000000;
            //System.out.println(dt + "ms");

            window.add(dt);
            // try {
            //     if (20 > dt) {
            //         Thread.sleep(20 - (long) dt);
            //     }
                
            // } catch (InterruptedException e) {
            //     e.printStackTrace();
            // }
        }

        System.out.println("Average calculation time: " + window.getAverage() + "ms");
        double var = window.getVariance2();
        System.out.println("Calculation time variance: " + var + "ms^2");
        System.out.println("Calculation time standard deviation: " + Math.sqrt(var) + "ms");
    }
}