package frc.robot.Utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;

public class tools {
    public static double minThreshold(double value, double min) {
        return Math.abs(value) < min ? 0 : value;
    }

    public static double getAngleError(double targetAngleRadians, double currentAngle) {
        return MathUtil.inputModulus(
                MathUtil.inputModulus(targetAngleRadians, 0, 2 * Math.PI)
                        - MathUtil.inputModulus(currentAngle, 0, 2 * Math.PI),
                -Math.PI, Math.PI);
    }

    public static double getRotationError(double targetAngleRotations, double currentAngle) {
        return MathUtil.inputModulus(
                MathUtil.inputModulus(targetAngleRotations, 0, 1)
                        - MathUtil.inputModulus(currentAngle, 0, 1),
                -0.5, 0.5);
    }

    public static boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent())
            return alliance.get() == DriverStation.Alliance.Red;
        return false;
    }

    public static double normalizedPolinominal(double x, int p, double n) {
        return n * x + (1-n) * Math.pow(x, p);
    }
}
