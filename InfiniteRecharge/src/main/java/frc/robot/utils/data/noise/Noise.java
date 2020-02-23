package frc.robot.utils.data.noise;

import org.ejml.simple.SimpleMatrix;

public class Noise {
    private final SimpleMatrix G;
    private final SimpleMatrix Q;



    public Noise(SimpleMatrix G, SimpleMatrix Q) {
        this.G = G;
        this.Q = Q;
    }



    public SimpleMatrix getG() { return G; }
    public SimpleMatrix getQ() { return Q; }

    public SimpleMatrix getCovariance() {
        return G.mult(Q).mult(G.transpose());
    }
}