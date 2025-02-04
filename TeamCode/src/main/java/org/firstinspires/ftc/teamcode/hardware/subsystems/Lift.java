package org.firstinspires.ftc.teamcode.hardware.subsystems;

import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.Lift.*;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {

    // debug - DELETE when lift works
    public int incrmital = 0;

    private DcMotorEx lower;
    private DcMotorEx middle;
    private DcMotorEx upper;

    public double error;
    private final int encPort;
    public double target = 0;
    private double liftState = 0;
    private double power = 0;
    private double manualPower = 0.0;

    public enum LIFT {
        RETURN(0),
        INTAKE(intake),
        LOW_FRONTLOAD(low_frontload),
        LOW(smallPole),
        MID(middlePole),
        HIGH(highPole);

        double ticks;

        LIFT(double ticks) {
            this.ticks = ticks;
        }

        public double getTicks() {
            return ticks;
        }
    }

    public enum STATE {
        MANUAL,
        RUNNING,
        STOP,
        IDLE,
    }

    public STATE state = STATE.IDLE;

    public Lift(HardwareMap hardwareMap) {
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

        encPort = lower.getPortNumber();
    }

    double previousPower = 0.0;
    public void setPower(double power) {
        if(previousPower != power) {
            lower.setPower(power);
            middle.setPower(power);
            upper.setPower(power);
            previousPower = power;
        }
    }

    public double getTarget() {
        return target;
    }

    public void setManuelPower(double manualPower) {
        this.manualPower = manualPower;
    }

    public double getState() {
        return liftState;
    }

    public double getError() {
        return error;
    }

    public boolean atTarget() {
        return Math.abs(error) < tolerance;
    }

    public void setTarget(double target) {
        state = STATE.RUNNING;
        this.target = target;
    }

    public void setTarget(int cycle) {
        switch (cycle) {
            case 0:
                setTarget(250);
                break;
            case 1:
                setTarget(210);
                break;
            case 2:
                setTarget(180);
                break;
            case 3:
                setTarget(150);
                break;
            case 4:
                setTarget(120);
                break;
            case 5:
                setTarget(90);
                break;
        }
    }

    public void setTarget(LIFT target) {
        state = STATE.RUNNING;
        this.target = target.getTicks();
    }

    public void stop() {
        state = STATE.STOP;
    }

    public void updateLift() {
        if(error < 0 && !atTarget()) {
            power = downSpeed;
        }
        else if(error > 0 && error < 200 && !atTarget()) {
            power = 0.7;
        }
        else if(error > 0 && !atTarget()) {
            power = 1;
        }
        else {
            power = Kg;
            state = STATE.STOP;
        }
    }

    public void loop(LynxModule.BulkData data) {

        incrmital++;

        liftState = data.getMotorCurrentPosition(encPort);
        error = target - liftState;

        if(Math.abs(manualPower) > 0.01) state = STATE.MANUAL;

        switch (state) {
            case MANUAL:
                setPower(manualPower + Kg);
                if(manualPower == 0.0) state = STATE.STOP;
                break;
            case RUNNING:
                updateLift();
                setPower(power);
                break;
            case STOP:
                setPower(Kg);
                state = STATE.IDLE;
            case IDLE:
                // do nothing
                break;
        }
    }
}
