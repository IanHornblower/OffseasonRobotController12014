package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.config.Config;

import com.outoftheboxrobotics.photoncore.Neutrino.RevColorSensor.RevColorSensorV3Ex;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.RobotConstants;
import org.firstinspires.ftc.teamcode.hardware.interfaces.Subsystem;
import org.firstinspires.ftc.teamcode.util.PID.PIDController;
import org.firstinspires.ftc.teamcode.util.motionprofile.MotionProfile;
import org.firstinspires.ftc.teamcode.util.motionprofile.MotionProfileGenerator;
import org.firstinspires.ftc.teamcode.util.motionprofile.MotionState;

import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.Faga.Claw.*;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.Faga.Fourbar.*;

@Config
public class Faga implements Subsystem {
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
    RevColorSensorV3Ex autoGrab;

    // Motion Profile for Fourbar

    private MotionProfile profile;

    public double previous_target = 5;

    VoltageSensor batteryVoltageSensor;
    ElapsedTime time;
    ElapsedTime voltageTimer;
    double voltage;
    public double manPower = 0.0;
    public boolean auto = true;

    public static enum STATE {
        ACTIVE,
        MANUEL,
        DISABLE,
        RESTING
    }

    STATE state = STATE.ACTIVE;

    double desiredFourbarPosition = 0;
    PIDController fourbarController = new PIDController(RobotConstants.Faga.Fourbar.kP, RobotConstants.Faga.Fourbar.kI, RobotConstants.Faga.Fourbar.kD);

    MotionState targetState;

    public Faga(HardwareMap hardwareMap) {
        // Hardware Map
        fourbar = hardwareMap.get(DcMotorEx.class, "fourbar");
        fourbar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fourbar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        time = new ElapsedTime();
        voltageTimer = new ElapsedTime();
        voltage = batteryVoltageSensor.getVoltage();

        leftArticulation = hardwareMap.get(Servo.class, "leftArt");
        rightArticulation = hardwareMap.get(Servo.class, "rightArt");

        claw = hardwareMap.get(Servo.class, "claw");
        autoGrab = hardwareMap.get(RevColorSensorV3Ex.class, "autograb");

        //claw = hardwareMap.get(Servo.class, "claw");
    }

    private double prevManPower = 0.0;

    @Override
    public void periodic() {
        switch (state) {
            case ACTIVE:
                updateFourbar();
                break;
            case MANUEL:
                if(manPower != prevManPower) {
                    fourbar.setPower(manPower);
                    prevManPower = manPower;
                }
                break;
            case DISABLE:
                fourbar.setPower(0.0);
                state = STATE.RESTING;
                break;
            case RESTING:
                // do literally nothing
                break;
        }


    }

    public void activate() {
        state = STATE.ACTIVE;
    }

    public void disable() {
        state = STATE.DISABLE;
    }

    public void manual() {
        state = STATE.MANUEL;
    }

    public void DEPRICATED_SET_TO_REST() {
        state = STATE.RESTING;
    }

    public double mp_target;
    public void updateFourbar() {
        double encPosition = fourbar.getCurrentPosition();
        double ff = (sine(calc(encPosition)) * -kCos);

        //PhotonCore.EXPANSION_HUB.getBulkData().getMotorCurrentPosition(fourbar.getPortNumber());

        if(auto) {

            if (desiredFourbarPosition != previous_target) {
                profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(previous_target, 0), new MotionState(desiredFourbarPosition, 0), max_v, max_a);
                time.reset();
                previous_target = desiredFourbarPosition;
            }

            if (voltageTimer.seconds() > 5) {
                voltage = batteryVoltageSensor.getVoltage();
                voltageTimer.reset();
            }

            targetState = profile == null ? new MotionState(0, 0) : profile.get(time.seconds());
            mp_target = targetState.getX();
            double pid = fourbarController.calculate(encPosition, mp_target);
            double fvfa = (kV * targetState.getV()) + (kA * targetState.getA());

            double output = (pid + ff + fvfa) / voltage * 12.0;


            fourbar.setPower(output);
        }
        else {
            fourbar.setPower(manPower + ff);
        }

    }

    public void setClawPosition(double position) {
        claw.setPosition(position);
    }

    public void updatePIDF() {
        fourbarController.setPID(kP, kI, kD);
        //fourbarControllerCustom = new CustomPID(kP, kI, kD, kCos);
    }

    public void setFourbarPosition(double position) {
        this.desiredFourbarPosition = position;
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
        setArticulation(RobotConstants.Faga.Articulator.intake);
    }

    public void articulateOuttake() {
        setArticulation(RobotConstants.Faga.Articulator.outtake);
    }

    public void articulateOuttakeAuto() {
        setArticulation(RobotConstants.Faga.Articulator.outtakeAuto);
    }

    public void returnToIntake() {
        fourbarController.setPID(-0.0008, 0.0, 0.0);
        kA = 0.00000015;
        kV = -0.000015;
        max_a = 8000;
        max_v = 15000;
        setFourbarPosition(250);
        articulateIntake();
    }

    public void setToOutake() {
        fourbarController.setPID(-0.00095, kI, kD);
        kA = -0.00000075;
        kV = -0.000018;
        max_a = 4500;
        max_v = 8000;
        setFourbarPosition(3800);
        articulateOuttake();
    }

    public void setToOutakeSmall() {
        fourbarController.setPID(-0.00095, kI, kD);
        kA = -0.00000075;
        kV = -0.000018;
        max_a = 4500;
        max_v = 8000;
        setFourbarPosition(3800);
        setArticulation(RobotConstants.Faga.Articulator.outtakeSmall);
    }

    public void setToOutakeAuto() {
        fourbarController.setPID(-0.00095, kI, kD);
        kA = -0.00000075;
        kV = -0.000018;
        max_a = 4500;
        max_v = 8000;
        setFourbarPosition(3800);
    }

    public void setToPrime() {
        fourbarController.setPID(-0.00095, 0, 0);
        kA = -0.00000075;
        kV = -0.000018;
        max_a = 4500;
        max_v = 8000;
        setFourbarPosition(3050);
        articulateIntake();
    }

    public void setToPrimeTele() {
        fourbarController.setPID(-0.00095, 0, 0);
        kA = -0.00000075;
        kV = -0.000018;
        max_a = 4500;
        max_v = 8000;
        setFourbarPosition(2450);
        articulateIntake();
    }

    public void setToPrimeAuto() {
        fourbarController.setPID(-0.0008, 0.0, 0.0);
        kA = 0.00000015;
        kV = -0.000015;
        max_a = 8000;
        max_v = 12000;
        setFourbarPosition(3050);

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

