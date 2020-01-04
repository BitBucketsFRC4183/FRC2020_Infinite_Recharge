package frc.robot.utils.control.encoder;

public class QuadratureEncoder extends SensorType {
    /*
     * For quadrature encoders, the ticks per revolution depends on the
     * type of encoder. TPR = 4 * CPR (quad comes from the 4)
     * where CPR is the internal counts per revolution to the
     * encoder.
     */

    /** Type of encoder */
    public enum EncoderType {
        AMT (2048); // AMT-201 and AMT-102 seem to have this, so probably all...



        private final int CPR;
        private EncoderType(int cpr) {
            CPR = cpr;
        }

        private int getCPR() {
            return CPR;
        }
    }



    private final int CPR;

    public QuadratureEncoder(EncoderType encoderType) {
        super(4 * encoderType.getCPR());

        CPR = encoderType.getCPR();
    }



    public int getCPR() {
        return CPR;
    }
}