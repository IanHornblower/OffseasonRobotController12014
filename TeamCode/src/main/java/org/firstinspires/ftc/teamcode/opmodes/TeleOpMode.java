package org.firstinspires.ftc.teamcode.opmodes;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TeleOpMode extends LinearOpMode {

    Thread stopThread;

    public void initOpMode() {

    }

    public void initLoopOpMode() throws InterruptedException {

    }

    public void startOpMode() {

    }

    public void updateOpMode() throws InterruptedException {

    }

    public void stopOpMode() {

    }

    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        PhotonCore.enable();
        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL); // make sure to always clear the bulk cache or im sure it will crash :)
        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.CONTROL_HUB.clearBulkCache();
        PhotonCore.EXPANSION_HUB.clearBulkCache();
        PhotonCore.experimental.setSinglethreadedOptimized(false); // idk i have like 5 threads :(
                                                                   // If crashing is an issue put everyhing into the
                                                                   // IMU thread to only have 2 threads running instead of
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        stopThread = new Thread(()-> {
            if(isStopRequested()) {
                stopOpMode();
                requestOpModeStop();
                stop();
            }
        });

        stopThread.start();

        initOpMode();

        while (opModeInInit() && !isStopRequested()) {
            initLoopOpMode();
        }

        waitForStart();
        startOpMode();

        while (opModeIsActive() && !isStopRequested()) {
            updateOpMode();

            PhotonCore.CONTROL_HUB.clearBulkCache();
            PhotonCore.EXPANSION_HUB.clearBulkCache();
        }
    }
}
