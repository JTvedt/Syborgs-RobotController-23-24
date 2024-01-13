package org.firstinspires.ftc.teamcode.util.math;

public class MathUtils {
    public static final double TWO_PI = Math.PI * 2;
    public static final double HALF_PI = Math.PI / 2;

    public static double normalizeAngle(double angle) {
        return angle - TWO_PI * Math.floor((angle + Math.PI) / TWO_PI);
    }

    public static double round(double num, int digits) {
        return Math.round(num * Math.pow(10, digits)) / Math.pow(10, digits);
    }

    public static double normalize(double value, double lowIn, double highIn, double lowOut, double highOut) {
        return (Math.max(Math.min(value, highIn), lowIn) - lowIn) / (highIn - lowIn) * (highOut - lowOut) + lowOut;
    }
}
