package org.firstinspires.ftc.teamcode.opmodes.comp.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.RobotConstants;
import org.firstinspires.ftc.teamcode.hardware.subsystems.SensorArmy;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;


@Autonomous(name = "Switch Auto Color Tracking", group = "!")
public class SwitchTracking extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Switch Auto Color");
        telemetry.addData("Current Color", AutoConst.autoTrackingColor.toString());
        telemetry.update();

        waitForStart();

        if(AutoConst.autoTrackingColor == SensorArmy.Color.RED) AutoConst.autoTrackingColor = SensorArmy.Color.BLUE;
        else AutoConst.autoTrackingColor = SensorArmy.Color.RED;

    }
}
