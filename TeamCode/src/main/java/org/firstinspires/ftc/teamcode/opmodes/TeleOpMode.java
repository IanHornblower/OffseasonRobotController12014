package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.hardware.Globals;

public class TeleOpMode extends JWOpMode {

    public GamepadEx operator;

    @Override
    public void initOpMode() {
        Globals.USING_AUTO_GRAB = true;

        super.initOpMode();


        operator = new GamepadEx(gamepad2);
    }

    @Override
    public void initLoopOpMode() throws InterruptedException {
        super.initLoopOpMode();

        robot.sensorArmy.startAutoGrabThread(this, true);
        telemetry.addData("AutoGrab", robot.sensorArmy.isAutoGrabActive());
        telemetry.update();
    }
}
