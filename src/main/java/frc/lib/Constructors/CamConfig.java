package frc.lib.Constructors;

import edu.wpi.first.math.geometry.Transform3d;

public class CamConfig {

    public final String name;
    public final Transform3d robotToCam;

    public final int width;
    public final int height;

    public CamGain gains = null;
    public CamGain rotGains = null;

    public CamConfig(String name, Transform3d robotToCam, int width, int height) {
        this.name = name;
        this.robotToCam = robotToCam;
        this.width = width;
        this.height = height;
    }

    public CamConfig(String name, Transform3d robotToCam, int width, int height, CamGain gains, CamGain rotGains) {
        this.name = name;
        this.robotToCam = robotToCam;
        this.width = width;
        this.height = height;
        this.gains = gains;
        this.rotGains = rotGains;
    }

    public record CamGain(double kDistance, double kAlpha, double kVelocity, double kOmega) {}
}