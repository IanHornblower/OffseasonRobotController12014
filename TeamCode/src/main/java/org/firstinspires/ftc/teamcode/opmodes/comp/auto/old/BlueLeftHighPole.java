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
import org.firstinspires.ftc.teamcode.hardware.subsystems.Camera;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Lift;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.roadrunner.Path;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

@Disabled
@Autonomous
public class BlueLeftHighPole extends LinearOpMode {
    Robot robot;
    ElapsedTime timer;
    ElapsedTime matchTimer;

    Pose2d start = new Pose2d(-39, -62, Math.toRadians(-90));

    Vector2d cycle1 = new Vector2d(-30, -0.8);
    double cycle1ang = Math.toRadians(35);
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

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(hardwareMap, telemetry);
        robot.camera.init();

        robot.drive.setPoseEstimate(start);
        robot.faga.resetFourbarEncoder();
        robot.faga.clawClose();

        TrajectorySequence toHighPoleAndStack = robot.drive.trajectorySequenceBuilder(start)
                .resetTimer(timer)
                .setReversed(true)

                .initCone(robot, 0.38)

                .splineTo(new Vector2d(-37, -36), Math.toRadians(90),
                        Path.augmentVelocity(65),
                        Path.augmentAcceleration(60))
                .resetConstraints()
                .splineTo(new Vector2d(-31, -6), Math.toRadians(29.5))

                .dropPreload(robot, 270)

                .setReversed(false)

                .splineTo(new Vector2d(-57, -13),Math.toRadians(180))
                .build();

        atConeStack = toHighPoleAndStack::end;

       // TrajectorySequence toCyclePole = robot.drive.trajectorySequenceBuilder(toHighPoleAndStack.end())
        TrajectorySequence toCyclePole = robot.drive.trajectorySequenceBuilder(atConeStack.get())
                .resetTimer(timer)
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(0.0, ()-> {
                    robot.lift.setModeManuel();
                    robot.lift.setPower(-0.2);
                    robot.faga.setFourbarPosition(robot.faga.getFourbar().getCurrentPosition() - 650);
                    //robot.lift.setPosition(robot.lift.getPosition() - 200);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.10, ()-> {
                    robot.faga.clawClose();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.15, ()-> {
                    robot.lift.setPower(0.0);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.35, ()-> {
                    robot.lift.setPosition(Lift.highPole-10);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.45, ()-> {
                    robot.lift.setModeAutomatic();
                })
                //.UNSTABLE_addTemporalMarkerOffset(0.55, ()-> robot.faga.setArticulation(RobotConstants.Faga.Articulator.autoStack))
                .UNSTABLE_addTemporalMarkerOffset(0.85, ()-> {
                    robot.faga.articulateOuttakeAuto();
                    robot.faga.setToOutakeAuto();
                })

                .waitSeconds(0.65)
                .setReversed(true)

                .splineTo(cycle1, cycle1ang, Path.augmentVelocity(45), Path.augmentAcceleration(35))

                .dropPreload(robot, 260)

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
               .build();

        TrajectorySequence parkRight = robot.drive.trajectorySequenceBuilder(atConeStack.get())
                .resetTimer(timer)
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0.3, ()-> robot.lift.setManuealPower(0.0))
                .back(45,
                        Path.augmentVelocity(80),
                        Path.augmentAcceleration(80))

                .build();

        timer.reset();
        while (opModeInInit() && !isStopRequested()) {
            robot.camera.runInInit();

            location = robot.camera.getSleeveLocation();

            telemetry.addData("Sleeve Location", robot.camera.getSleeveLocation().toString());
            telemetry.addData("Time since Init", timer.seconds());
            telemetry.update();
        }

        robot.drive.followTrajectorySequenceAsync(toHighPoleAndStack);
        robot.camera.shutdown();
        waitForStart();

        matchTimer.reset();
        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("toWallState", robot.tw_state.toString());
            telemetry.addData("auto state", matchState.toString());
            telemetry.addData("match time", matchTimer.seconds());
            telemetry.addData("cycle", cycle);
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
                        robot.drive.followTrajectorySequenceAsync(toCyclePole);
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
                                    robot.lift.setPosition(50);
                                    break;

                            }
                            matchState = Auto.TO_CONE_STACK;
                        }
                    }
                    break;
                case GRAB_LAST_CONE:
                    robot.lift.setPosition(0);
                    if(!grabLast && robot.tw_state.equals(Robot.TO_WALL.END)) {
                        if(location == Camera.State.MIDDLE) robot.grabConeBeacon();
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
