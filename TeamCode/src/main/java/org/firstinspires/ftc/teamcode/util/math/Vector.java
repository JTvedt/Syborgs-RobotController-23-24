package org.firstinspires.ftc.teamcode.util.math;

public class Vector {
    private double x;
    private double y;

    public Vector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getX() { return x; }
    public double getY() { return y; }

    public void setX(double x) { this.x = x; }
    public void setY(double y) { this.y = y; }

    public Vector rotate(double theta) {
        Vector v = new Vector(this.x, this.y);

        // Basically just matrix multiplication
        x = v.x * Math.cos(theta) + v.y * Math.cos(theta + MathUtils.HALF_PI);
        y = v.x * Math.sin(theta) + v.y * Math.sin(theta + MathUtils.HALF_PI);

        return this;
    }

    public Vector stretch(double x, double y) {
        this.x *= x;
        this.y *= y;
        return this;
    }

    public Vector add(Vector other) {
        this.x += other.x;
        this.y += other.y;
        return this;
    }

    public Vector subtract(Vector other) {
        this.x -= other.x;
        this.y -= other.y;
        return this;
    }

    public Vector multiply(double magnitude) {
        return this.stretch(magnitude, magnitude);
    }

    public double hypot() {
        return Math.hypot(x, y);
    }
}
