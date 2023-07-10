package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.util.Precision;
import org.firstinspires.ftc.teamcode.hardware.Robot;

public class JWOpMode extends OpModeEx {
    public ElapsedTime timer;
    public Robot robot;
    double hz = 0;
    double loop = 0;
    double loopTime = 0;
    double prevLoopTime = 0;

    @Override
    public void initOpMode() {
        robot = new Robot(getHardwareMap(), telemetry);
        timer = new ElapsedTime();
    }

    /**
     * Make sure to add telemetry.update(); to end of the loop
     * @throws InterruptedException
     */
    @Override
    public void initLoopOpMode() throws InterruptedException {
        telemetry.addData("Time since Init", Precision.round(timer.seconds(), 2));
    }

    @Override
    public void updateOpMode() throws InterruptedException {
        robot.periodic();

        loop = System.nanoTime();
        loopTime = (loop - prevLoopTime);
        hz = 1000000000 / loopTime;
        prevLoopTime = loop;

        telemetry.addData("hz", hz);
        telemetry.addData("loop time", loopTime/1e+6);
    }

    @Override
    public void stopOpMode() {
        robot.stop();
    }
}
