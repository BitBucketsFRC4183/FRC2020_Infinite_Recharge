package frc.robot.utils.data.filters;

import frc.robot.utils.data.DoubleDataWindow;



public class RunningAverageFilter {
    private DoubleDataWindow dw;



    public RunningAverageFilter(int length) {
        dw = new DoubleDataWindow(length);
    }



    public double calculate(double data) {
        dw.add(data);

        return dw.getAverage();
    }
}