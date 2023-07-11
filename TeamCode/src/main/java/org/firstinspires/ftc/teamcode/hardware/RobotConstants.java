package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.hardware.subsystems.SensorArmy.Color.RED;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.hardware.subsystems.SensorArmy;

public class RobotConstants {

    public static org.firstinspires.ftc.teamcode.hardware.subsystems.SensorArmy.Color trackingColor = RED;

    public static class Faga {
        @Config
        public static class Articulator {
            public static double intake = 0.50; // was 0.57
            public static double outtake = 0.41; // was 0.48
            public static double frontLoad = 0.65;
            public static double outtakeAuto = 0.45;
            public static double outtakeSmall = 0.46;
            public static double autoStack = 0.54;
        }
        @Config
        public static class Claw {
            public static double open = 0;
            public static double transfer = 0.5;
            public static double close = 1;
        }

        @Config
        public static class Fourbar {
            public static double outtake = 3700;
            public static double frontLoad = 2000;
            public static double regularKp = 0.00031;
            public static double frontloadKp = 0.00045;

            public static double tolerance = 100;
            public static double ticksPerRev = 8192;
            public static double max_v = 9000;
            public static double max_a = 8000;
            public static double kP = 0.00031, kD = 0.0, kCos = 0.12, kI = 0.0, kSin = 0;
            public static double kV = 0.00003, kA = 0.000042;
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

    @Config
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

    @Config
    public static class Lift {
        public static double Kg = 0.065; // Tune

        public static double tolerance = 15;
        public static double downSpeed = -0.42; // -0.35

        public static double intake = 130;
        public static double low_frontload = 482;
        public static double smallPole = 165; // was 150
        public static double middlePole = 540;
        public static double highPole = 1000; // 990
    }
}
