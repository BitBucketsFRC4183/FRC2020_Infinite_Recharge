package frc.robot.utils.data.filters;



public abstract class Filter<T> {
    public abstract T calculate(T measurement);
}