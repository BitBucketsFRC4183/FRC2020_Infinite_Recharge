package frc.robot.utils.data.noise;

import org.ejml.simple.SimpleMatrix;

/*
 * ==================================
 * 2020 Code Review Party Information
 * 
 * Feel free to ignore this file
 * ==================================
 */
public class Noise {
    private final SimpleMatrix G;
    private final SimpleMatrix Q;



    public Noise(SimpleMatrix G, SimpleMatrix Q) {
        this.G = G;
        this.Q = Q;
    }

    public Noise(SimpleMatrix Q) {
        this(Q, SimpleMatrix.identity(Q.numCols()));
    }



    public SimpleMatrix getG() { return G; }
    public SimpleMatrix getQ() { return Q; }

    public SimpleMatrix getCovariance() {
        return G.mult(Q).mult(G.transpose());
    }
}