package org.firstinspires.ftc.teamcode.roadrunner.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class Jameson {
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
