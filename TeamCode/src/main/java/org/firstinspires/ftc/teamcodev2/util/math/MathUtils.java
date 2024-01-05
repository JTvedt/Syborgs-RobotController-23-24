package org.firstinspires.ftc.teamcodev2.util.math;

public class MathUtils {
    public static final double TWO_PI = Math.PI * 2;
    public static final double HALF_PI = Math.PI / 2;

    public static double normalizeAngle(double angle) {
        return angle - TWO_PI * Math.floor((angle + Math.PI) / TWO_PI);
    }

    public static double round(double num, int digits) {
        return Math.round(num * Math.pow(10, digits)) / Math.pow(10, digits);
    }

    public static double normalize(double value, double lowBoundary, double highBoundary, double low, double high) {
        return Math.max(Math.min((value - lowBoundary) / (highBoundary - lowBoundary) * (high - low) + low, high), low);
    }
}
