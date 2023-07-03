package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.hardware.subsystems.SensorArmy.Color.RED;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.hardware.subsystems.SensorArmy;

public class RobotConstants {

    public static org.firstinspires.ftc.teamcode.hardware.subsystems.SensorArmy.Color trackingColor = RED;

    public static class Faga {
        @Config
        public static class Articulator {
            public static double outerbound = 0.19;
            public static double innerbound = 0.81;
            public static double intake = 0.3;
            public static double outtake = 0.4;
            public static double outtakeAuto = 0.45;
            public static double outtakeSmall = 0.46;
            public static double autoStack = 0.32;
        }
        @Config
        public static class Claw {
            public static double open = 0;
            public static double transfer = 0.5;
            public static double close = 1;
        }
        @Config
        public static class Fourbar {
            public static double max_v = 15000;
            public static double max_a = 8000;
            //public static double gearRatio = 5.89;
            public static double ticksPerRev = 8192;
            //public static double kP = 0.00038, kD = 0.0000000085, kCos = 0.25, kI = 0.0;
            public static double kP = -0.0002, kD = 0.0, kCos = 0.15, kI = 0.02, kSin = 0;
            public static double kV = -0.000056, kA = -0.000012;
        }
    }

    public static class Eyelids {
        public static double l_up = 0.95;
        public static double l_down = 0.27;

        public static double r_up = 0.25;
        public static double r_down = 0.93;

        public static double r_correct = 0.8;
        public static double l_correct = 0.4;
    }

    public static class SensorArmy {

        public static double Q = 0.35, R = 2, N = 2;
        public static  double kC = 0.8;
        public static double b = 5; // in IN distance between IR sensors

        public static double activeLowerboundBLUE = 550; //1200
        public static double activeLowerboundRED = 550; //1200

        public static double upperCutOffBlue = 1100; // 1600
        public static double lowerCutOffBlue = 550; // 1100

        public static double upperCutOffRed = 1200; // 1600
        public static double lowerCutOffRed = 550; // 1100

        public static double kP = 5;
    }

}
