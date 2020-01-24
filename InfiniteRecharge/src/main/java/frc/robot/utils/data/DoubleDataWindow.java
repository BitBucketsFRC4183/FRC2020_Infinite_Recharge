package frc.robot.utils.data;



public class DoubleDataWindow extends DataWindow<Double> {
    double sum = 0;



    public DoubleDataWindow(int length) {
        super(length);
    }
    


    @Override
    public void add(Double elm) {
        sum -= data.get(next);
        sum += elm;

        super.add(elm);
    }



    public double getAverage() {
        return sum / numFilled;
    }

    public double getVariance() {
        double avg = getAverage();

        double var = 0;

        for (int i = 0; i < numFilled; i++) {
            var += Math.pow(get(i) - avg, 2);
        }

        return var / numFilled;
    }
}