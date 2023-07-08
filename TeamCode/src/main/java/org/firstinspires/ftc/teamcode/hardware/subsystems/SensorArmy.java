package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.ThermalEquilibrium.homeostasis.Filters.Estimators.KalmanEstimator;
import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.KalmanFilter;
import com.acmerobotics.dashboard.config.Config;
import com.outoftheboxrobotics.photoncore.Neutrino.Rev2MSensor.Rev2mDistanceSensorEx;
import com.outoftheboxrobotics.photoncore.Neutrino.RevColorSensor.RevColorSensorV3Ex;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.checkerframework.checker.units.qual.K;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.interfaces.ColorState;
import org.firstinspires.ftc.teamcode.hardware.interfaces.Subsystem;

import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.SensorArmy.*;

import java.util.Arrays;

import javax.annotation.concurrent.GuardedBy;

@Config
public class SensorArmy implements Subsystem {
    public enum Color {
        RED,
        BLUE
    }

    private final Object distanceLock = new Object();
    @GuardedBy("distanceLock")
    private Rev2mDistanceSensorEx leftDistance, rightDistance;
    private Thread distanceThread;
    private double leftDistance_in, rightDistance_in;

    private final Object tapeLock = new Object();
    @GuardedBy("tapeLock")
    private RevColorSensorV3Ex l2, l1,  m, r1, r2;
    private Thread tapeThread;
    private ColorState l2_state, l1_state, m_state, r1_state, r2_state;
    private ColorState[] array;


    // TODO: do the autograb thread but rn it lame fr | LLLL
    //  Make as many of the subsystems non-periodic to allow for lower loop times
    //  Only ones that HAVE to be should be lift and fourbar (maybe neither if one of them is shut off in idle position)
    //  Make sure to disable the more resource intensive one (prolly fourbar) ie. lift raised up a bit while fourbar power is cut full
    private final Object autograbLock = new Object();
    @GuardedBy("autograbLock")
    private RevColorSensorV3Ex autoGrab;
    private Thread autoGrabThread;
    private ColorState autoGrab_state;
    KalmanFilter autograb = new KalmanFilter(Q,R,(int)N);

    private boolean[] sensorActive = new boolean[5];

    public Color folowingColor = Color.RED;

    public SensorArmy(HardwareMap hardwareMap) {
        leftDistance = hardwareMap.get(Rev2mDistanceSensorEx.class, "leftIR");
        rightDistance = hardwareMap.get(Rev2mDistanceSensorEx.class, "rightIR");
        leftDistance.setRangingProfile(Rev2mDistanceSensorEx.RANGING_PROFILE.HIGH_ACCURACY);
        rightDistance.setRangingProfile(Rev2mDistanceSensorEx.RANGING_PROFILE.HIGH_ACCURACY);

        l2 = hardwareMap.get(RevColorSensorV3Ex.class, "l2");
        l1 = hardwareMap.get(RevColorSensorV3Ex.class, "l1");
        m = hardwareMap.get(RevColorSensorV3Ex.class, "m");
        r1 = hardwareMap.get(RevColorSensorV3Ex.class, "r2");
        r2 = hardwareMap.get(RevColorSensorV3Ex.class, "r1");

        autoGrab = hardwareMap.get(RevColorSensorV3Ex.class, "autograb");
        autoGrab.setUpdateRate(RevColorSensorV3Ex.UPDATE_RATE.DEFAULT);

        l2.setUpdateRate(RevColorSensorV3Ex.UPDATE_RATE.HIGH_SPEED);
        l1.setUpdateRate(RevColorSensorV3Ex.UPDATE_RATE.HIGH_SPEED);
        m.setUpdateRate(RevColorSensorV3Ex.UPDATE_RATE.HIGH_SPEED);
        r1.setUpdateRate(RevColorSensorV3Ex.UPDATE_RATE.HIGH_SPEED);
        r2.setUpdateRate(RevColorSensorV3Ex.UPDATE_RATE.HIGH_SPEED);
    }

    public void startAutoOpMode(LinearOpMode opMode) {
        startDistanceThread(opMode);
        startTapeThread(opMode);
    }

    public void startTeleOpMode(LinearOpMode opMode) {

    }

    public void startAllOpMode(LinearOpMode opMode) {
        startAutoOpMode(opMode);
        startTeleOpMode(opMode);
    }

    public void startDistanceThread(LinearOpMode opMode) {
        if(Globals.USING_IR) {
            distanceThread = new Thread(()-> {
                while (!opMode.isStopRequested() && opMode.opModeIsActive()) {
                    synchronized (distanceLock) {
                        leftDistance_in = leftDistance.getDistance(DistanceUnit.INCH);
                        rightDistance_in = rightDistance.getDistance(DistanceUnit.INCH);
                    }
                }
            });
            distanceThread.start();
        }
    }

    public void startTapeThread(LinearOpMode opMode) {
        if(Globals.USING_TAPE_SENSORS) {
            tapeThread = new Thread(()-> {
                while (!opMode.isStopRequested() && opMode.opModeIsActive()) {
                    synchronized (tapeLock) {
                        l2_state = new ColorState(l2.red(), l2.green(), l2.blue(), l2.alpha(), l2.getLightDetected(), l2.getDistance(DistanceUnit.INCH));
                        l1_state = new ColorState(l1.red(), l1.green(), l1.blue(), l1.alpha(), l1.getLightDetected(), l1.getDistance(DistanceUnit.INCH));
                        m_state = new ColorState(m.red(), m.green(), m.blue(), m.alpha(), m.getLightDetected(), m.getDistance(DistanceUnit.INCH));
                        r1_state = new ColorState(r1.red(), r1.green(), r1.blue(), r1.alpha(), r1.getLightDetected(), r1.getDistance(DistanceUnit.INCH));
                        r2_state = new ColorState(r2.red(), r2.green(), r2.blue(), r2.alpha(), r2.getLightDetected(), r2.getDistance(DistanceUnit.INCH));

                        array = new ColorState[]{l2_state, l1_state, m_state, r1_state, r2_state};

                        for(int i = 0; i < sensorActive.length; i++) {
                            double threshold;

                            if(folowingColor.equals(Color.RED)) {
                                threshold = (lowerCutOffRed + upperCutOffRed) / 2;

                                sensorActive[i] = array[i].blue() > threshold;
                            }
                            else {
                                threshold = (lowerCutOffBlue + upperCutOffBlue) / 2;

                                sensorActive[i] = array[i].red() > threshold;
                            }
                        }

                    }
                }
            });
            tapeThread.start();
        }
    }

    public void startAutoGrabThread(LinearOpMode opMode) {
        if(Globals.USING_AUTO_GRAB) {
            autoGrabThread = new Thread(()-> {
                while (!opMode.isStopRequested() && opMode.opModeIsActive()) {
                    synchronized (autograbLock) {
                        //
                    }
                }
            });
            autoGrabThread.start();
        }
    }

    public ColorState[] getArray() {
        return array;
    }

    public boolean[] getActive() {
        return sensorActive;
    }

    public void setFollowingColor(Color c) {
        folowingColor = c;

    }

    public double getSmoothedAutoGrab(DistanceUnit distanceUnit) {
        return autograb.estimate(autoGrab.getDistance(distanceUnit));
    }

    public double getLeftIR() {
        return leftDistance_in;
    }

    public double getRightIR() {
        return rightDistance_in;
    }

    public double getDistanceToWall() {
        return (getLeftIR() + getRightIR()) / 2.0;
    }

    // Split this into multiple methods whilst this may work
    // it really should be separated to allow for better debugging and
    // better testing of each individual parts for angular compensation
    public double getDistanceOffset() {
        double a = Math.abs(getRightIR() - getLeftIR());
        double c = Math.sqrt(a*a + b*b);

        double covariance = (c / b) * kC; // Strengthen Loop
        double angle = Math.asin(a/c);

        if(getLeftIR() > getRightIR()) return angle * covariance;
        else return -angle * covariance;
    }

    // Mayble multithread this as well // forloop and arraystream could slow down loop times
    public double getPosition() {
        double[] cut = new double[5];

        for(int i = 0; i < 5; i++) {
            if(folowingColor.equals(Color.BLUE)) cut[i] = Range.clip(array[i].blue() - lowerCutOffBlue, 0, Double.POSITIVE_INFINITY);
            else cut[i] = Range.clip(array[i].red()  - lowerCutOffRed, 0, Double.POSITIVE_INFINITY);
        }

        cut[3] *= -1;   cut[4] *= -1;   cut[2] = 0;

        double sum = Arrays.stream(cut).sum();
        double effectiveRange;

        if(folowingColor.equals(Color.BLUE)) effectiveRange = upperCutOffBlue - lowerCutOffBlue;
        else effectiveRange = upperCutOffRed - lowerCutOffRed;

        // Return normalized position in range from about 0-1  -> PID should be less than 1
        // it ends up being around 2.5 or so but who cares :) so long as the reloc value is tuned it don matter
        return sum / effectiveRange;
    }

    public boolean isOnLine() {
        return !sensorActive[0] && !sensorActive[4] && sensorActive[2];
    }

    public void periodic() {
    }
}
