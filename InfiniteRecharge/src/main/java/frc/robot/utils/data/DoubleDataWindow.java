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
        return sum / LENGTH;
    }
}