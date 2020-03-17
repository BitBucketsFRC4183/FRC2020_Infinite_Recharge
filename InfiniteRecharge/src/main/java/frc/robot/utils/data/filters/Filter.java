package frc.robot.utils.data.filters;



// a filter that removes some unwanted traits from a signal of type T
public abstract class Filter<T> {
    // calculate the filtered value given a measurement
    public abstract T calculate(T measurement);
}