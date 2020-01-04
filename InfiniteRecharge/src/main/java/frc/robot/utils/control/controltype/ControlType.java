package frc.robot.utils.control.controltype;

public enum ControlType {
    Position (Variable.Position),
    MotionMagic (Variable.Position),
    Velocity (Variable.Velocity),
    Current (Variable.Current),
    Voltage (Variable.Voltage);

    public static enum Variable {
        Position,
        Velocity,
        Current,
        Voltage
    }



    private final Variable VAR;

    private ControlType(Variable v) {
        this.VAR = v;
    }

    public Variable getVariable() {
        return VAR;
    }
}