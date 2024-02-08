package frc.robot.utils;

// Shot parameter
public class ShotParameter {

    // Variables
    public final double angle;
    public final double leftrpm;
    public final double rightrpm;

    // Constructor
    public ShotParameter(double angle, double leftrpm, double rightrpm) {
        this.angle = angle;
        this.leftrpm = leftrpm;
        this.rightrpm = rightrpm;
    }

    // Method equals
    public boolean equals(ShotParameter other) {
        return Math.abs(this.angle - other.angle) < 0.1 &&
                Math.abs(this.leftrpm - other.leftrpm) < 0.1 &&
                Math.abs(this.rightrpm - other.leftrpm) < 0.1;
    }

    // Method to interpolate
    public ShotParameter interpolate(ShotParameter end, double t) {
        return new ShotParameter(
                lerp(angle, end.angle, t),
                lerp(leftrpm, end.leftrpm, t),
                lerp(rightrpm, end.rightrpm, t));
    }

    // Method lerp
    private double lerp(double y2, double y1, double t) {
        return y1 + (t * (y2 - y1));
    }

}