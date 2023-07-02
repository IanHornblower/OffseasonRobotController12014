package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.ThermalEquilibrium.homeostasis.Filters.Estimators.KalmanEstimator;
import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.KalmanFilter;
import com.acmerobotics.dashboard.config.Config;
import com.outoftheboxrobotics.photoncore.Neutrino.Rev2MSensor.Rev2mDistanceSensorEx;
import com.outoftheboxrobotics.photoncore.Neutrino.RevColorSensor.RevColorSensorV3Ex;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.checkerframework.checker.units.qual.K;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.interfaces.Subsystem;

import java.util.Arrays;

@Config
public class SensorArmy implements Subsystem {
    public Rev2mDistanceSensorEx leftDistance;
    public Rev2mDistanceSensorEx rightDistance;

    public RevColorSensorV3Ex l2;
    public RevColorSensorV3Ex l1;
    public RevColorSensorV3Ex m;
    public RevColorSensorV3Ex r1;
    public RevColorSensorV3Ex r2;

    public RevColorSensorV3Ex autoGrab;

    public RevColorSensorV3Ex[] sensorsArray;
    public boolean[] sensorActive = new boolean[5];

    public  double Q = 0.35, R = 2, N = 2;
    public  double kC = 0.8;
    double b = 5; // in IN distance between IR sensors

    public static double activeLowerboundBLUE = 550; //1200
    public static double activeLowerboundRED = 550; //1200

    public static double upperCutOffBlue = 1100; // 1600
    public static double lowerCutOffBlue = 550; // 1100


    public static double upperCutOffRed = 1200; // 1600
    public static double lowerCutOffRed = 550; // 1100

    public static double kP = 5;

    KalmanFilter autograb = new KalmanFilter(Q,R,(int)N);

    KalmanFilter distanceSensors = new KalmanFilter(Q, R, (int)N);

    KalmanFilter colorSensors = new KalmanFilter(Q, R, (int)N);

    public Color folowingColor = Color.RED;

    private TrackingMethod tm = TrackingMethod.PID;

    public enum TrackingMethod {
        PID,
        BILINEAR,
        SIMPLE
    }

    public enum Color {
        RED,
        BLUE
    }

    public SensorArmy(HardwareMap hardwareMap) {
        leftDistance = hardwareMap.get(Rev2mDistanceSensorEx.class, "leftIR");
        rightDistance = hardwareMap.get(Rev2mDistanceSensorEx.class, "rightIR");
        leftDistance.setRangingProfile(Rev2mDistanceSensorEx.RANGING_PROFILE.HIGH_SPEED);
        rightDistance.setRangingProfile(Rev2mDistanceSensorEx.RANGING_PROFILE.HIGH_SPEED);

        l2 = hardwareMap.get(RevColorSensorV3Ex.class, "l2");
        l1 = hardwareMap.get(RevColorSensorV3Ex.class, "l1");
        m = hardwareMap.get(RevColorSensorV3Ex.class, "m");
        r1 = hardwareMap.get(RevColorSensorV3Ex.class, "r2");
        r2 = hardwareMap.get(RevColorSensorV3Ex.class, "r1");

        autoGrab = hardwareMap.get(RevColorSensorV3Ex.class, "autograb");
        autoGrab.setUpdateRate(RevColorSensorV3Ex.UPDATE_RATE.HIGH_SPEED);

        sensorsArray = new RevColorSensorV3Ex[] {l2, l1, m, r1, r2};

        for(RevColorSensorV3Ex sensor : sensorsArray) {
            sensor.setUpdateRate(RevColorSensorV3Ex.UPDATE_RATE.HIGH_SPEED);
        }
    }

    public void updateKalmanFilter() {
        autograb = new KalmanFilter(Q,R,(int)N);

    }

    public void setTrackingMethod(TrackingMethod tm) {
        this.tm = tm;
    }

    public void setFollowingColor(Color c) {
        folowingColor = c;

    }


    public double getSmoothedAutoGrab(DistanceUnit distanceUnit) {
        return autograb.estimate(autoGrab.getDistance(distanceUnit));
    }

    public double getLeftIR() {
        return leftDistance.getDistance(DistanceUnit.INCH);
    }

    public double getRightIR() {
        return rightDistance.getDistance(DistanceUnit.INCH);
    }

    public double getDistanceOffset() {
        double a = Math.abs(getRightIR() - getLeftIR());
        double c = Math.sqrt(a*a + b*b);

        double covariance = (c / b) * kC; // Strengthen Loop
        double angle = Math.asin(a/c);

        if(getLeftIR() > getRightIR()) return angle * covariance;
        else return -angle * covariance;
    }

    public double getDistanceToWall() {
        return (getLeftIR() + getRightIR()) / 2.0;
    }

    public double getError() {
        double error = 0;
        switch (tm) {
            case PID:
                double sum = 0.0;
                for (int i = 0; i < 5; i++) {
                    if(folowingColor.equals(Color.RED)) {
                        sum += (i - 2) * Range.clip(sensorsArray[i].red(), 600, 1900);
                    }
                    else {
                        sum += (i - 2) * Range.clip(sensorsArray[i].blue(), 600, 1900);
                    }

                }

                boolean anyActive = false;
                for(boolean active : sensorActive) {
                    if (active) {
                        anyActive = true;
                        break;
                    }
                }
                    if(folowingColor.equals(Color.RED)) {
                        error =  -sum / (
                                sensorsArray[0].red() +
                                        sensorsArray[1].red() +
                                        sensorsArray[2].red() +
                                        sensorsArray[3].red() +
                                        sensorsArray[4].red()
                        );
                    }
                    else {
                        error =  -sum / (
                                sensorsArray[0].blue() +
                                        sensorsArray[1].blue() +
                                        sensorsArray[2].blue() +
                                        sensorsArray[3].blue() +
                                        sensorsArray[4].blue()
                        );
                    }

                break;
            case BILINEAR:
                break;
            case SIMPLE:
                //if()
                break;
        }
        return error;
    }

    public double getPosition() {
        double[] cut = new double[5];

        for(int i = 0; i < 5; i++) {
            if(folowingColor.equals(Color.BLUE)) cut[i] = Range.clip(sensorsArray[i].blue() - lowerCutOffBlue, 0, Double.POSITIVE_INFINITY);
            else cut[i] = Range.clip(sensorsArray[i].red() - lowerCutOffRed, 0, Double.POSITIVE_INFINITY);
        }

        cut[3] *= -1;   cut[4] *= -1;   cut[2] = 0;

        double sum = Arrays.stream(cut).sum();
        double effectiveRange;

        if(folowingColor.equals(Color.BLUE)) effectiveRange = upperCutOffBlue - lowerCutOffBlue;
        else effectiveRange = upperCutOffRed - lowerCutOffRed;

        // Return normalized position in range from about 0-1  -> PID should be less than 1
        return sum / effectiveRange;
    }

    public double getOutput() {
        return getError() * kP;
    }

    public boolean isOnLine() {
        return !sensorActive[0] && !sensorActive[4] && sensorActive[2];
    }

    public void colorSensorUpdate() {
        for(int i = 0; i < sensorActive.length; i++) {
            if (folowingColor.equals(Color.RED)) {
                if(sensorsArray[i].red() > 1200) {
                    sensorActive[i] = true;
                }
                else {
                    sensorActive[i] = false;
                }
            }
            else {
                if(sensorsArray[i].blue() > 1200) {
                    sensorActive[i] = true;
                }
                else {
                    sensorActive[i] = false;
                }
            }

        }
    }

    public void periodic() {
        updateKalmanFilter();
        colorSensorUpdate();
    }
}
