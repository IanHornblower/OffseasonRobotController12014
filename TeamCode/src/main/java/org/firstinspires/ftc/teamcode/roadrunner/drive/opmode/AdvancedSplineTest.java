package org.firstinspires.ftc.teamcode.roadrunner.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Disabled
@Autonomous(group = "drive")
public class AdvancedSplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d initialPose = new Pose2d(-60, 36, 0);
        Vector2d first = new Vector2d(-36, 36);
        Vector2d second = new Vector2d(-36,12);
        Vector2d third = new Vector2d(-12,12);
        Vector2d fourth = new Vector2d(24,12);

        drive.setPoseEstimate(initialPose);

        waitForStart();

        if (isStopRequested()) return;

        Trajectory traj = drive.trajectoryBuilder(initialPose)
                .splineTo(first, calculateTangent(initialPose,first))
                .splineTo(second, calculateTangent(first,second))
                .splineTo(third, calculateTangent(second,third))
                .splineTo(fourth, calculateTangent(third,fourth))
                .build();

        drive.followTrajectory(traj);
    }

    public static double calculateTangent(Pose2d initialPosition, Pose2d finalPosition) {
        double xd = initialPosition.vec().getX() - finalPosition.vec().getX();
        double yd = initialPosition.vec().getY() - finalPosition.vec().getY();
        return Math.atan2(yd,xd) - Math.PI;
    }
    public static double calculateTangent(Vector2d initialPosition, Vector2d finalPosition) {
        double xd = initialPosition.getX() - finalPosition.getX();
        double yd = initialPosition.getY() - finalPosition.getY();
        return Math.atan2(yd,xd) - Math.PI;
    }
    public static double calculateTangent(Pose2d initialPosition, Vector2d finalPosition) {
        double xd = initialPosition.getX() - finalPosition.getX();
        double yd = initialPosition.getY() - finalPosition.getY();
        return Math.atan2(yd,xd) - Math.PI;
    }
    public static double calculateTangent(Vector2d initialPosition, Pose2d finalPosition) {
        double xd = initialPosition.getX() - finalPosition.getX();
        double yd = initialPosition.getY() - finalPosition.getY();
        return Math.atan2(yd,xd) - Math.PI;
    }
}
