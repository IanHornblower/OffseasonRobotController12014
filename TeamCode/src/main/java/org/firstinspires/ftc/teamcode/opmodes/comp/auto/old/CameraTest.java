package org.firstinspires.ftc.teamcode.opmodes.comp.auto.old;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.RobotConstants;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Lift;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.roadrunner.Path;

import java.util.function.Supplier;

@Disabled
@Autonomous
public class CameraTest extends LinearOpMode {
    Robot robot;
    ElapsedTime timer;

    Pose2d start = new Pose2d(-39, -62, Math.toRadians(-90));

    @Override
    public void runOpMode() throws InterruptedException {
        timer = new ElapsedTime();

        robot = new Robot(hardwareMap, telemetry);
        robot.camera.init();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());



        timer.reset();
        while (opModeInInit() && !isStopRequested()) {
            robot.camera.runInInit();
            telemetry.addData("camera pos", robot.camera.getSleeveLocation().toString());
            telemetry.addData("Time since Init", timer.seconds());
            telemetry.update();
        }

        waitForStart();
        robot.camera.shutdown();

        while (opModeIsActive() && !isStopRequested()) {

        }
    }
}
