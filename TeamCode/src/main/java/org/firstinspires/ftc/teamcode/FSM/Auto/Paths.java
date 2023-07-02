package org.firstinspires.ftc.teamcode.FSM.Auto;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class Paths {

    public static SampleMecanumDrive d;

    public static void setDrive(SampleMecanumDrive d) {
        Paths.d = d;
    }

    public static class NormalHigh {

        public static class Left {
            public static Pose2d start = new Pose2d(-39, -62, Math.toRadians(-90));

            public static TrajectorySequence dropPreload = d.trajectorySequenceBuilder(start)
                    .setReversed(true)
                    .splineTo(new Vector2d(-35.9, -36), Math.toRadians(90))
                    .splineTo(new Vector2d(-31, -7.2), Math.toRadians(32))
                    .build();

            public static TrajectorySequence toConeStackFromPreload = d.trajectorySequenceBuilder(dropPreload.end())
                    .setReversed(false)
                    .splineTo(new Vector2d(-55, -11),Math.toRadians(180))
                    .build();
            public static TrajectorySequence toConeStackFromCycle = d.trajectorySequenceBuilder(dropPreload.end())
                    .setReversed(false)
                    .splineTo(new Vector2d(-55, -12),Math.toRadians(180))
                    .build();
            public static TrajectorySequence toHighPole = d.trajectorySequenceBuilder(new Pose2d(-62, -12, Math.toRadians(180)))
                    .setReversed(true)
                    .splineTo(new Vector2d(-31, -7.2), Math.toRadians(32))
                    .build();

            public static TrajectorySequence parkLeft = d.trajectorySequenceBuilder(toHighPole.end())
                    .strafeLeft(10)
                    .build();

            public static TrajectorySequence parkMid = d.trajectorySequenceBuilder(toHighPole.end())
                    .forward(10)
                    .build();

            public static TrajectorySequence parkRight = d.trajectorySequenceBuilder(toHighPole.end())
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
