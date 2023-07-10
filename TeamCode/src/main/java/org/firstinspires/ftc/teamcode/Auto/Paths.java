package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.outoftheboxrobotics.photoncore.ReflectionUtils;

import org.firstinspires.ftc.teamcode.FSM.FiniteStateMachine;
import org.firstinspires.ftc.teamcode.FSM.actions.Delay;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.RobotConstants;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Faga;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Lift;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class Paths {

    public static SampleMecanumDrive d;
    public static Robot robot;

    public static void setDrive(Robot robot) {
        Paths.d = robot.drive;
        Paths.robot = robot;
    }

    public static class SingletonActions {
        public static Delay initCone = new Delay(()-> {
            robot.faga.setFourbarPosition(RobotConstants.Faga.Fourbar.outtake);
            robot.faga.articulateOuttake();
        });
        public static Delay toHigh = new Delay(()-> robot.lift.setTarget(Lift.LIFT.HIGH.getTicks()-15));

        public static Delay returnLiftToIntake = new Delay(()-> robot.lift.setTarget(250));

        public static Delay returnLift = new Delay(()-> robot.lift.setTarget(0));


        public static FiniteStateMachine[] Singletons = new FiniteStateMachine[] {initCone,toHigh,returnLift,returnLiftToIntake};
    }

    public static class NormalHigh {

        public static class Left {
            public static Pose2d start = new Pose2d(-39, -62, Math.toRadians(-90));

            public static TrajectorySequence dropPreload = d.trajectorySequenceBuilder(start)
                    .setReversed(true)
                    .UNSTABLE_addTemporalMarkerOffset(0.3, ()-> {
                        robot.lift.setTarget(Lift.LIFT.HIGH.getTicks()-10);
                        robot.faga.articulateOuttake();
                        robot.faga.setFourbarPosition(RobotConstants.Faga.Fourbar.outtake);
                    })
                    .splineTo(new Vector2d(-34.5, -36), Math.toRadians(90.00))
                    .splineTo(new Vector2d(-25.5, -5.5), Math.toRadians(55.00))
                    .UNSTABLE_addTemporalMarkerOffset(-0.2, robot.faga::clawOpen)
                    .build();

            public static TrajectorySequence toConeStackFromPreload = d.trajectorySequenceBuilder(dropPreload.end())
                    .setReversed(false)
                    .UNSTABLE_addTemporalMarkerOffset(0.2, ()-> {
                        robot.faga.setFourbarPosition(0);
                        robot.faga.articulateIntake();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.5, ()-> {
                        robot.lift.setTarget(300);
                        robot.lift.target = 300;
                        robot.lift.state = Lift.STATE.RUNNING;
                    })
                    .splineTo(new Vector2d(-45.00, -12.80), Math.toRadians(180.00))
                    .splineTo(new Vector2d(-56.00, -12.80), Math.toRadians(180.00))

                    .build();
            public static TrajectorySequence toConeStackFromCycle = d.trajectorySequenceBuilder(dropPreload.end())
                    .setReversed(false)
                    .UNSTABLE_addTemporalMarkerOffset(0.2, ()-> {
                          robot.faga.setFourbarPosition(0);
                        robot.faga.articulateIntake();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.5, ()-> {
                        robot.lift.setTarget(220);
                        robot.lift.target = 220;
                        robot.lift.state = Lift.STATE.RUNNING;
                    })
                    .splineTo(new Vector2d(-56, -12),Math.toRadians(180))
                    .build();
            public static TrajectorySequence toHighPole = d.trajectorySequenceBuilder(new Pose2d(-62, -12, Math.toRadians(180)))
                    .setReversed(true)
                    .UNSTABLE_addTemporalMarkerOffset(0.5, ()-> {
                        robot.faga.articulateOuttake();
                        robot.faga.setManuelPower(0.0);
                        robot.faga.setFourbarPosition(RobotConstants.Faga.Fourbar.outtake);
                        robot.faga.target = RobotConstants.Faga.Fourbar.outtake;
                        robot.faga.state = Faga.STATE.RUNNING;
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.8, ()-> {
                        //robot.lift.setTarget(Lift.LIFT.HIGH.getTicks()-15);
                        robot.lift.target = 1030;
                        robot.lift.state = Lift.STATE.RUNNING;
                    })
                    .splineTo(new Vector2d(-30.15, -0.99), Math.toRadians(32.00))


                    .build();

            public static TrajectorySequence parkLeft = d.trajectorySequenceBuilder(toHighPole.end())
                    .addTemporalMarker(()-> {
                        robot.faga.articulateOuttake();
                        robot.faga.setManuelPower(0.0);
                        robot.faga.setFourbarPosition(0);
                        robot.faga.target = 0;
                        robot.faga.state = Faga.STATE.RUNNING;
                        robot.faga.articulateIntake();
                        robot.faga.clawOpen();

                        robot.lift.target = 0;
                        robot.lift.state = Lift.STATE.RUNNING;
                    })
                    .splineTo(new Vector2d(-51.80, -13.96), Math.toRadians(200.00))
                    .splineTo(new Vector2d(-57, -33), Math.toRadians(-90.00))
                    .build();

            public static TrajectorySequence parkMid = d.trajectorySequenceBuilder(toHighPole.end())
                    .addTemporalMarker(()-> {
                        robot.faga.articulateOuttake();
                        robot.faga.setManuelPower(0.0);
                        robot.faga.setFourbarPosition(0);
                        robot.faga.target = 0;
                        robot.faga.state = Faga.STATE.RUNNING;
                        robot.faga.articulateIntake();
                        robot.faga.clawOpen();

                        robot.lift.target = 0;
                        robot.lift.state = Lift.STATE.RUNNING;
                    })
                    .splineTo(new Vector2d(-32.0, -33.00), Math.toRadians(-90.00))
                    .build();

            public static TrajectorySequence parkRight = d.trajectorySequenceBuilder(toHighPole.end())
                     .addTemporalMarker(()-> {
                         robot.faga.articulateOuttake();
                         robot.faga.setManuelPower(0.0);
                         robot.faga.setFourbarPosition(0);
                         robot.faga.target = 0;
                         robot.faga.state = Faga.STATE.RUNNING;
                         robot.faga.articulateIntake();
                         robot.faga.clawOpen();

                         robot.lift.target = 0;
                         robot.lift.state = Lift.STATE.RUNNING;
                     })
                    .strafeRight(10)
                    .build();

            public static TrajectorySequence getHighPole(Pose2d updatedLocal) {
                toHighPole.setStart(updatedLocal);
                return toHighPole;
            }

            public static TrajectorySequence[] getParkPositions() {
                return new TrajectorySequence[] {parkLeft, parkMid, parkRight};
            }
        }

        public static class Right {

        }
    }

}
