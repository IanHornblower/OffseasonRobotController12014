package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.subsystems.SensorArmy;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.PID.PIDController;

import java.util.Arrays;

@Config
@TeleOp(name = "Line Center Tuner")
public class LineCenterTester extends LinearOpMode {
    public static boolean active = false;
    public static double wallDistance = 2.2;
    public static double forwardSpeed = 0.25;

    public static double lineP = 0.047, lineI = 0, lineD = 0;
    public static double headingP = -0.8, headingI = 0.05, headingD = 0;

    public static PIDController heading = new PIDController(lineP,lineI,lineD);
    public static PIDController line = new PIDController(headingP,headingI,headingD);

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive d = new SampleMecanumDrive(hardwareMap);
        SensorArmy a = new SensorArmy(hardwareMap);

        d.setPoseEstimate(new Pose2d(0,0,Math.toRadians(90)));

        telemetry.addLine("Init Done");
        telemetry.update();

        a.setFollowingColor(SensorArmy.Color.RED);

        waitForStart();

        double startAngle = d.getPoseEstimate().getHeading();

        heading.setSetPoint(0);
        line.setSetPoint(0);

        while (opModeIsActive() && !isStopRequested()) {
            d.update();
            a.periodic();

            line.setPID(lineP,lineI,lineD);
            heading.setPID(headingP,headingI,headingD);

            if(active) {
                double x = forwardSpeed;
                double y = line.calculate(a.getPosition());
                double h = heading.calculate(AngleUnit.normalizeRadians(startAngle - d.getPoseEstimate().getHeading()));

                if(a.getDistanceToWall() < wallDistance) {
                    x = 0.0;
                }

                //d.setDrivePower(new Pose2d(0, y, h));
                d.setDrivePower(new Pose2d(x, y, h));
            }
            else {
                d.stop();
            }

            telemetry.addData("position", a.getPosition());
            telemetry.addData("activy", Arrays.toString(a.sensorActive));
            telemetry.addData("is on line", a.isOnLine());
            telemetry.addData("distance to wall", a.getDistanceToWall());
            telemetry.update();

        }


    }
}
