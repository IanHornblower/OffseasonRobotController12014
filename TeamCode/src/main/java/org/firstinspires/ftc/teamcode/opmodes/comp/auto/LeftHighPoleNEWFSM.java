package org.firstinspires.ftc.teamcode.opmodes.comp.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.actions.FollowTrajectorySequence;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Camera;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Mogus;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.roadrunner.Path;

import java.util.function.Supplier;

@Autonomous(group = "!")
public class LeftHighPoleNEWFSM extends LinearOpMode {
    Robot robot;
    ElapsedTime timer;
    ElapsedTime matchTimer;

    Mogus mogus;

    Pose2d start = new Pose2d(-39, -62, Math.toRadians(-90));

    Vector2d cycle1 = new Vector2d(-29.55, -0.85);
    double cycle1ang = Math.toRadians(33.5);
    Vector2d cycle2 = new Vector2d(-29.55, -1.5);
    double cycle2ang = Math.toRadians(33.5);
    Vector2d cycle3 = new Vector2d(-29.55, -2.7);
    double cycle3ang = Math.toRadians(33.5);
    Vector2d cycle4 = new Vector2d(-29.55, -2.7);
    double cycle4ang = Math.toRadians(33.5);


    Supplier<Pose2d> atConeStack;
    Camera.State location = Camera.State.LEFT;

    int cycle = 1;

    enum Auto {
        PRELOAD,
        TO_CONE_STACK,
        TO_HIGH_POLE,
        GRAB_LAST_CONE,
        PARK,
        END
    }

    Auto matchState = Auto.PRELOAD;
    boolean grabLast = false;
    boolean parked = false;

    @Override
    public void runOpMode() throws InterruptedException {
        timer = new ElapsedTime();
        matchTimer = new ElapsedTime();

        mogus = new Mogus();

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(hardwareMap, telemetry);
        robot.camera.init();

        robot.drive.setPoseEstimate(start);
        robot.faga.resetFourbarEncoder();
        robot.faga.clawClose();

        robot.sensorArmy.setFollowingColor(AutoConst.autoTrackingColor);

        TrajectorySequence toHighPoleAndStack = robot.drive.trajectorySequenceBuilder(start)
                .resetTimer(timer)
                .setReversed(true)

                .initCone(robot, 0.6)

                .splineTo(new Vector2d(-35.9, -36), Math.toRadians(90),
                        Path.augmentVelocity(65),
                        Path.augmentAcceleration(60))
                .resetConstraints()
                .splineTo(new Vector2d(-31, -7.2), Math.toRadians(32))

                .dropPreload(robot, 260)

                .setReversed(false)

                .splineTo(new Vector2d(-57, -13),Math.toRadians(180))
                .build();

        atConeStack = toHighPoleAndStack::end;

       // TrajectorySequence toCyclePole = robot.drive.trajectorySequenceBuilder(toHighPoleAndStack.end())
        TrajectorySequence toCyclePole = robot.drive.trajectorySequenceBuilder(atConeStack.get())
                .resetTimer(timer)
                .setReversed(false)

                .grabCone(robot)

                .waitSeconds(0.65)

                .setReversed(true)
                .splineTo(cycle1, cycle1ang, Path.augmentVelocity(45), Path.augmentAcceleration(35))
                .dropPreload(robot, 270)

                .setReversed(false)
                .splineTo(new Vector2d(-60, -12),Math.toRadians(180),
                        Path.augmentVelocity(42),
                        Path.augmentAcceleration(34))
                .build();

        TrajectorySequence toCyclePole2 = robot.drive.trajectorySequenceBuilder(atConeStack.get())
                .resetTimer(timer)
                .setReversed(false)

                .grabCone(robot)

                .waitSeconds(0.65)

                .setReversed(true)
                .splineTo(cycle2, cycle2ang, Path.augmentVelocity(45), Path.augmentAcceleration(35))
                .dropPreload(robot, 270)

                .setReversed(false)
                .splineTo(new Vector2d(-60, -12),Math.toRadians(180),
                        Path.augmentVelocity(42),
                        Path.augmentAcceleration(34))
                .build();

        TrajectorySequence toCyclePole3 = robot.drive.trajectorySequenceBuilder(atConeStack.get())
                .resetTimer(timer)
                .setReversed(false)

                .grabCone(robot)

                .waitSeconds(0.65)

                .setReversed(true)
                .splineTo(cycle3, cycle3ang, Path.augmentVelocity(45), Path.augmentAcceleration(35))
                .dropPreload(robot, 270)

                .setReversed(false)
                .splineTo(new Vector2d(-60, -12),Math.toRadians(180),
                        Path.augmentVelocity(42),
                        Path.augmentAcceleration(34))
                .build();

        TrajectorySequence toCyclePole4 = robot.drive.trajectorySequenceBuilder(atConeStack.get())
                .resetTimer(timer)
                .setReversed(false)

                .grabCone(robot)

                .waitSeconds(0.65)

                .setReversed(true)
                .splineTo(cycle4, cycle4ang, Path.augmentVelocity(45), Path.augmentAcceleration(35))
                .dropPreload(robot, 270)

                .setReversed(false)
                .splineTo(new Vector2d(-60, -12),Math.toRadians(180),
                        Path.augmentVelocity(42),
                        Path.augmentAcceleration(34))
                .build();



        TrajectorySequence parkMid = robot.drive.trajectorySequenceBuilder(atConeStack.get())
                .resetTimer(timer)
                .waitSeconds(0.2)
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(-0.3, ()-> robot.lift.setManuealPower(0.0))
                .back(19,
                        Path.augmentVelocity(80),
                        Path.augmentAcceleration(80))
                .UNSTABLE_addTemporalMarkerOffset(-0.3, ()-> robot.eyelids.up())
               .build();

        TrajectorySequence parkRight = robot.drive.trajectorySequenceBuilder(atConeStack.get())
                .resetTimer(timer)
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0.3, ()-> robot.lift.setManuealPower(0.0))
                .back(41,
                        Path.augmentVelocity(80),
                        Path.augmentAcceleration(80))
                .UNSTABLE_addTemporalMarkerOffset(-0.3, ()-> robot.eyelids.up())
                .build();

        timer.reset();
        while (opModeInInit() && !isStopRequested()) {
            robot.camera.runInInit();

            location = robot.camera.getSleeveLocation();

            telemetry.addData("Following Color", robot.sensorArmy.folowingColor.toString());
            telemetry.addData("Sleeve Location", robot.camera.getSleeveLocation().toString());
            telemetry.addData("Time since Init", timer.seconds());
            telemetry.update();
        }

        robot.drive.followTrajectorySequenceAsync(toHighPoleAndStack);
        robot.camera.shutdown();
        waitForStart();

        matchTimer.reset();
        while (opModeIsActive() && !isStopRequested()) {
            mogus.periodic();

            telemetry.addData("toWallState", robot.tw_state.toString());
            telemetry.addData("auto state", matchState.toString());
            telemetry.addData("match time", matchTimer.seconds());
            telemetry.addData("cycle", cycle);

            telemetry.addLine();
            telemetry.addLine();
            telemetry.addLine(mogus.get35pxTwerk());
            telemetry.update();

            robot.autoUpdate();

            switch (matchState) {
                case PRELOAD:
                    robot.drive.update();
                    if(timer.seconds() > toHighPoleAndStack.duration()-0.05) {
                        timer.reset();

                        robot.toWall();
                        matchState = Auto.TO_CONE_STACK;
                    }
                    break;
                case TO_CONE_STACK:
                    if(robot.tw_state.equals(Robot.TO_WALL.END)) {
                        matchState = Auto.TO_HIGH_POLE;
                        timer.reset();
                    }
                    break;
                case TO_HIGH_POLE:
                    robot.drive.update();
                    if(!robot.drive.isBusy()) {
                        switch (cycle) {
                            case 1:
                                robot.drive.followTrajectorySequenceAsync(toCyclePole);
                                break;
                            case 2:
                                robot.drive.followTrajectorySequenceAsync(toCyclePole2);
                                break;
                            case 3:
                                robot.drive.followTrajectorySequenceAsync(toCyclePole3);
                                break;
                            case 4:
                                robot.drive.followTrajectorySequenceAsync(toCyclePole4);
                                break;
                        }

                    }
                    if(timer.seconds() > toCyclePole.duration()) {
                        timer.reset();
                        cycle++;
                        if(cycle > 4) {
                            robot.toWall();
                            robot.lift.setPosition(0);
                            robot.forwardSpeed = 0.5;
                            matchState = Auto.GRAB_LAST_CONE;
                        }
                        else {
                            robot.toWall();
                            switch (cycle) {
                                case 2:
                                    robot.lift.setPosition(170);
                                    break;
                                case 3:
                                    robot.lift.setPosition(100);
                                    break;
                                case 4:
                                    robot.lift.setPosition(60);
                                    break;

                            }
                            matchState = Auto.TO_CONE_STACK;
                        }
                    }
                    break;
                case GRAB_LAST_CONE:
                    robot.lift.setPosition(0);
                    if(!grabLast && robot.tw_state.equals(Robot.TO_WALL.END)) {
                        //if(location == Camera.State.MIDDLE) robot.grabConeBeacon();
                        grabLast = true;
                        matchState = Auto.PARK;
                    }
                     break;
                case PARK:
                    switch (location) {
                        case LEFT:
                            robot.lift.setModeManuel();
                            robot.lift.setManuealPower(-0.15);
                            matchState = Auto.END;
                            break;
                        case RIGHT:
                            robot.lift.setModeManuel();
                            robot.lift.setManuealPower(-0.15);

                            if(!robot.drive.isBusy() && !parked) {
                                robot.drive.followTrajectorySequenceAsync(parkRight);
                                parked = true;
                            }
                            if(!robot.drive.isBusy() && parked) {
                                matchState = Auto.END;
                            }
                            break;
                        case MIDDLE:
                            robot.lift.setModeManuel();
                            robot.lift.setManuealPower(-0.15);

                            if(!robot.drive.isBusy() && !parked) {
                                robot.drive.followTrajectorySequenceAsync(parkMid);
                                parked = true;
                            }
                            if(!robot.drive.isBusy() && parked) {
                                matchState = Auto.END;
                            }
                            break;
                    }
                    robot.drive.update();

                    break;
                case END:
                    robot.lift.setPower(0.0);

                    //sleep(1000000);
                    robot.drive.stop();
                    robot.faga.stopFourbar();
                    robot.lift.setPower(0);
                    stop();
                    break;
            }
        }
    }
}
