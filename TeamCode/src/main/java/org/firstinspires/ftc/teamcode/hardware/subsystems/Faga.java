package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotConstants;
import org.firstinspires.ftc.teamcode.hardware.interfaces.Subsystem;
import org.firstinspires.ftc.teamcode.util.PID.PIDController;
import org.firstinspires.ftc.teamcode.util.motionprofile.MotionProfileGenerator;
import org.firstinspires.ftc.teamcode.util.motionprofile.MotionState;

import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.Faga.Claw.*;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.Faga.Fourbar.*;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.Faga.Articulator.*;

@Config
public class Faga {
    // F: Fourbar
    // A: Articulation
    // G: Grabbing
    // A: Apparatus

    // Fourbar
    DcMotorEx fourbar;

    // Articulation
    Servo leftArticulation;
    Servo rightArticulation;

    // Claw
    Servo claw;

    private double error;
    private final int encPort;
    public double target = 0;
    private double fourbarState = 0;
    private double power = 0;
    private double manualPower = 0.0;
    private double ff = 0.0;

    public static enum STATE {
        MANUAL,
        RUNNING,
        LOCK,
        STOP,
        IDLE
    }

    public STATE state = STATE.IDLE;

    PIDController fourbarController = new PIDController(RobotConstants.Faga.Fourbar.kP, RobotConstants.Faga.Fourbar.kI, RobotConstants.Faga.Fourbar.kD);

    public Faga(HardwareMap hardwareMap) {
        // Hardware Map
        fourbar = hardwareMap.get(DcMotorEx.class, "fourbar");
        fourbar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fourbar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightArticulation = hardwareMap.get(Servo.class, "leftArt");
        leftArticulation = hardwareMap.get(Servo.class, "rightArt");

        claw = hardwareMap.get(Servo.class, "claw");

        encPort = fourbar.getPortNumber();
    }

    public void loop(LynxModule.BulkData data) {
        fourbarState = data.getMotorCurrentPosition(encPort);
        ff = sine(calc(fourbarState)) * kCos;
        power = fourbarController.calculate( fourbarState, target) + ff;
        error = fourbarController.getPositionError();
        if(atTarget()) state = STATE.LOCK;
        if((Math.abs(manualPower) > 0.01) && !Globals.IS_AUTO) state = STATE.MANUAL;

        switch (state) {
            case MANUAL:
                setPower(manualPower + ff);
                if(manualPower == 0.0 || Globals.IS_AUTO) state = STATE.LOCK;
                break;
            case RUNNING:
                setPower(power);
                break;
            case LOCK:
                setPower(ff);
                state = STATE.IDLE;
                break;
            case STOP:
                setPower(0.0);
                state = STATE.IDLE;
                break;
            case IDLE:

                //if(fourbar.getPower() != 0 || data.isMotorOverCurrent(encPort)) {
                //    fourbar.setPower(0.0);
                //}
                // do nothing
                break;
        }


    }

    double previousPower = 0.0;
    public void setPower(double power) {
        if(previousPower != power) {
            fourbar.setPower(power);
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
        return fourbarState;
    }

    public double getError() {
        return error;
    }

    public boolean atTarget() {
        return Math.abs(error) < tolerance;
    }

    public void stop() {
        state = STATE.STOP;
    }

    public void setClawPosition(double position) {
        claw.setPosition(position);
    }

    public void updatePIDF() {
        fourbarController.setPID(kP, kI, kD);
    }

    public void setFourbarPosition(double position) {
        state = STATE.RUNNING;
        this.target = position;
    }

    public double calc(double enc) {
        double ticksPerRadian = ticksPerRev / (Math.PI * 2.0);

        return (enc / ticksPerRadian) + Math.toRadians(55);
    }

    public void resetFourbarEncoder() {
        fourbar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fourbar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setArticulation(double position) {
        leftArticulation.setPosition(position);
        rightArticulation.setPosition(position);
    }

    public void clawClose() {
        setClawPosition(close);
    }

    public void clawOpen() {
        setClawPosition(open);
    }

    public void clawTransfer() {
        setClawPosition(transfer);
    }

    public void articulateIntake() {
        setArticulation(intake);
    }

    public void articulateOuttake() {
        setArticulation(RobotConstants.Faga.Articulator.outtake);
    }

    public void articulateFrontLoad() {
        setArticulation(frontLoad);
    }

    //Bhaskara I's sine approximation  // goob
    private double sine(double radians) {
        double pi = Math.PI;
        double tau = pi * 2;
        double toDegrees = 180/pi;
        double h = 8100;

        double g = radians - pi;

        if(radians <= pi && radians > 0) { // 0-180
            return ((radians * toDegrees) * (180 - radians*toDegrees))/h;
        }
        else if(radians > pi && radians <= tau) {
            return -((g* toDegrees) * (180 - g*toDegrees))/h;
        }
        else {
            return 0.0;
        }
    }

    public void stopFourbar() {
        fourbar.setPower(0.0);
    }

    public DcMotorEx getFourbar() {
        return fourbar;
    }
}

