package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.interfaces.Subsystem;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.Eyelids.*;

@Config
public class Eyelids implements Subsystem{

    public Servo left;
    public Servo right;

    public boolean isEyelidDown = false;

    public Eyelids(HardwareMap hardwareMap) {
        left = hardwareMap.get(Servo.class, "leftEyelid");
        right = hardwareMap.get(Servo.class, "rightEyelid");
    }

    public void leftUp() {
        left.setPosition(l_up);
    }
    public void rightUp() {
        right.setPosition(r_up);
    }
    public void leftDown() {
        left.setPosition(l_down);
    }
    public void rightDown() {
        right.setPosition(r_down);
    }

    public void rightCorrect() {
        right.setPosition(r_correct);
    }
    public void leftCorrect() {
        left.setPosition(l_correct);
    }

    public void up() {
        leftUp();
        rightUp();
        isEyelidDown = false;
    }

    public void down() {
        leftDown();
        rightDown();
        isEyelidDown = true;
    }

    public void correct() {
        leftCorrect();
        rightCorrect();
    }

    @Override
    public void periodic() {

    }
}
