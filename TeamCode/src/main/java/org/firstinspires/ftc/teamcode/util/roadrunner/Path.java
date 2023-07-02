package org.firstinspires.ftc.teamcode.util.roadrunner;

import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;


public class Path {
    public static TrajectoryVelocityConstraint augmentVelocity(double velocity) {
        return SampleMecanumDrive.getVelocityConstraint(velocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    }

    public static TrajectoryVelocityConstraint augmentVelocity(double velocity, double angularVelocity) {
        return SampleMecanumDrive.getVelocityConstraint(velocity, angularVelocity, DriveConstants.TRACK_WIDTH);
    }

    public static TrajectoryAccelerationConstraint augmentAcceleration(double acceleration) {
        return SampleMecanumDrive.getAccelerationConstraint(acceleration);
    }
}
