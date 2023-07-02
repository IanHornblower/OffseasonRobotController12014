package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.interfaces.Subsystem;

public class Intake implements Subsystem {

    CRServo left, right;

    public Intake(HardwareMap hardwareMap) {
        left = hardwareMap.get(CRServo.class, "leftIntake");
        right = hardwareMap.get(CRServo.class, "rightIntake");
    }

    public void setLeftPower(double power) {
        left.setPower(-power);
    }

    public void setRightPower(double power) {
        right.setPower(power);
    }

    public void setPower(double power) {
        setRightPower(power);
        setLeftPower(power);
    }

    public void start() {
        setPower(1);
    }
    public void reverse() {
        setPower(-1);
    }
    public void stop() {
        setPower(0);
    }

    @Override
    public void periodic() {

    }
}
