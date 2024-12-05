package frc.lib.Constructors;

public class PivotMotorConfig {
    public final double kP;
    public final double kI;
    public final double kD;
    public final double kS;
    public final double kV;
    public final double kG;

    public PivotMotorConfig(double kP, double kI, double kD, double kS, double kV, double kG) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kS = kS;
        this.kV = kV;
        this.kG = kG;
    }
}
