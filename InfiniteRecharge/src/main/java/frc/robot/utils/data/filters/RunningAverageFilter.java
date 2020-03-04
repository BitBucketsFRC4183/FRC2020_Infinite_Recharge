package frc.robot.utils.data.filters;

import frc.robot.utils.data.DoubleDataWindow;



public class RunningAverageFilter extends Filter<Double> {
    private DoubleDataWindow dw;



    public RunningAverageFilter(int length) {
        dw = new DoubleDataWindow(length);
    }



    @Override
    public Double calculate(Double data) {
        dw.add(data);

        return getAverage();
    }

    public double getAverage() {
        return dw.getAverage();
    }
}