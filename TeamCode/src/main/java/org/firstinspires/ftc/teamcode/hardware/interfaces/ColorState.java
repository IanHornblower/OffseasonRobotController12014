package org.firstinspires.ftc.teamcode.hardware.interfaces;

public class ColorState {
    private double red, green, blue, alpha, luminocity, distance;
    public ColorState(double red, double green, double blue, double alpha, double luminosity, double distance) {
        this.red = red;
        this.green = green;
        this.blue = blue;
        this.alpha = alpha;
        this.luminocity = luminosity;
        this.distance = distance;
    }

    public double getLuminocity() {
        return luminocity;
    }

    public double getAlpha() {
        return alpha;
    }

    public double blue() {
        return blue;
    }

    public double green() {
        return green;
    }

    public double red() {
        return red;
    }

    public double getDistance() {
        return distance;
    }
}
