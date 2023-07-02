package org.firstinspires.ftc.teamcode.opmodes.comp.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;


@TeleOp(name = "Retract Odom")
public class Retract extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive s = new SampleMecanumDrive(hardwareMap);

        telemetry.addLine("Start To Retract Odom");
        telemetry.update();

        waitForStart();

        s.retract();
    }
}
