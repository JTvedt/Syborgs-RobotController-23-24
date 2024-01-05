package org.firstinspires.ftc.teamcodev2.util.math;

public class DistanceUnit {
    public static final double TICK = 1;
    public static final double CM = 537.7 / 6.201;
    public static final double INCH = CM * 2.54;
    public static final double TILE = INCH * 24;

    public static int toTicks(double value, double unit) {
        return (int)(unit * value);
    }
}
