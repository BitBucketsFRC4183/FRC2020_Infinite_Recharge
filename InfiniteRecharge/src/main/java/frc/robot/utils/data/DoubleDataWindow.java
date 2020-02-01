package frc.robot.utils.data;



public class DoubleDataWindow extends DataWindow<Double> {
    double sumX = 0;
    double sumX2 = 0;



    public DoubleDataWindow(int length) {
        super(length);
    }
    


    @Override
    public void add(Double elm) {
        if (isFilled()) {
            double x = data.get(next);

            sumX -= x;
            sumX2 -= x * x;
        }

        sumX += elm;
        sumX2 += elm * elm;

        super.add(elm);
    }



    public double getAverage() {
        return sumX / numFilled;
    }

    public double getVariance() {
        double avg = getAverage();

        double var = 0;

        for (int i = 0; i < numFilled; i++) {
            var += Math.pow(get(i) - avg, 2);
        }

        return var / numFilled;
    }

    // unbiased, using Bessel correction
    public double getVariance2() {
        if (numFilled <= 1) {
            return 0;
        }

        // numerically unstable but we're ignoring that :))
        // thanks Bessel
        return (sumX2/numFilled - Math.pow(getAverage(), 2)) * (1 + 1.0/(numFilled - 1));
    }
}