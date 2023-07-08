package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.interfaces.Subsystem;

@Config
public class LiftOld implements Subsystem {

    DcMotorEx lower;
    DcMotorEx middle;
    DcMotorEx upper;

    //TODO: finish subsystem and motor config
    public static double Kg = 0.09; // Was 0.07

    public double position = 0;
    public static double smallPole = 210; // was 150
    public static double middlePole = 475;
    public static double highPole = 930;
    public static double highPoleBroken = 1220;

    public double error;

    public enum LIFT {
        RETURN(0),
        LOW(smallPole),
        MID(middlePole),
        HIGH(highPole),
        SUPERHIGH(highPoleBroken);

        double ticks;

        LIFT(double ticks) {
            this.ticks = ticks;
        }

        public double getTicks() {
            return ticks;
        }
    }

    public boolean manual = false;

    public void setModeManuel() {
        manual = true;
    }

    public void setModeAutomatic() {
        manual = false;
    }

    public LiftOld(HardwareMap hardwareMap) {
        lower = hardwareMap.get(DcMotorEx.class, "lower");
        middle = hardwareMap.get(DcMotorEx.class, "middle");
        upper = hardwareMap.get(DcMotorEx.class, "upper");

        upper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        upper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lower.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        middle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        upper.setDirection(DcMotorSimple.Direction.REVERSE);
        lower.setDirection(DcMotorSimple.Direction.REVERSE);
        middle.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    double tolerance = 30;
    public static double downSpeed = -0.5; // -0.35

    public void runToPosition(double position) {
        error = position - getEncoderPosition();
        if(error < 0 && Math.abs(error) > tolerance) {
            setPower(downSpeed + manPower);
        }
        else if(error > 0 && error < 200 && Math.abs(error) > tolerance) {
            setPower(0.7 + manPower);
        }
        else if(error > 0 && Math.abs(error) > tolerance) {
            setPower(1 + manPower);
        }
        else {
            setPower(Kg + manPower);
        }
    }

    double previousPower = 0.0;

    public void setPower(double power) {
        if(previousPower == power) {
            //pass
        }
        else {
            lower.setPower(power);
            middle.setPower(power);
            upper.setPower(power);
            previousPower = power;
        }
    }

    public double getPosition() {
        return position;
    }

    public double getEncoderPosition() {
        return lower.getCurrentPosition();
    }

    public void setPosition(double position) {
        this.position = position;
    }

    double manPower = 0.0;
    public void setManuealPower(double power) {
        manPower = power;
    }

    public boolean isLiftDown() {
        //return !limitSwitch.getState();
        return Math.abs(getEncoderPosition()) < 30;
    }

    @Override
    public void periodic() {
        if(!manual) {
            runToPosition(position);
        }
        else {
            setPower(manPower);
        }
    }



}
