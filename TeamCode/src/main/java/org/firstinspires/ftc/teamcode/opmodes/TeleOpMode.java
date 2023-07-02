package org.firstinspires.ftc.teamcode.opmodes;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.PhotonCore;
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
        PhotonCore.CONTROL_HUB.clearBulkCache();
        PhotonCore.EXPANSION_HUB.clearBulkCache();
        PhotonCore.experimental.setSinglethreadedOptimized(true);

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
        }
    }
}
