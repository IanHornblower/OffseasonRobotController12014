package org.firstinspires.ftc.teamcode.util.PID;

import com.qualcomm.robotcore.util.ElapsedTime;

public class SquareRootController {
    ElapsedTime timer = new ElapsedTime();
    private double lasterror = 0;
    boolean hasRun = false;

    private double k,h,kD;
    private double reference;
    private double setPoint;
    private double measuredValue;

    public SquareRootController(double k, double h, double kD) {
        this.k = k;
        this.h = h;
        this.kD = kD;

        timer.reset();
    }

    public void setSetPoint(double sp) {
        setPoint = sp;
    }

    public double calculate(double state) {
        double error = reference - state;
        double output;
        if (error > 0) {
            output = Math.sqrt(error) * k + h;
        } else {
            output = -Math.sqrt(Math.abs(error)) * k - h;
        }

        if (!hasRun) {
            lasterror = error;
            hasRun = true;
        }

        double derivative =( error - lasterror) / timer.seconds();
        timer.reset();
        lasterror = error;

        return output + kD * derivative;
    }

    public double calculate(double pv, double sp) {
        setSetPoint(sp);
        return calculate(pv);
    }

    public void setCoefficients(double k, double h, double kD) {
        this.k = k;
        this.h = h;
        this.kD = kD;
    }
}
