package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.opmodes.TeleOpMode;

@Config
@TeleOp
public class HzTeleOpTesting extends TeleOpMode {
    double low = 999;
    double high = 0;
    double filtered = 0;
    double average = 0;
    double sum = 0;
    double count = 0;

    public static boolean camera = false;
    public static boolean eyelids = false;
    public static boolean Faga = false;
    public static boolean intake = false;
    public static boolean drive = false;
    public static boolean lift = false;
    public static boolean mogus = false;
    public static boolean sensorArray = false;
    public static boolean ENABLE_ROBOT = false;

    double prevLoopTime = 0;
    double hz = 0;

    Robot robot;



    @Override
    public void initOpMode() {
        Globals.USING_TAPE_SENSORS = true;
        Globals.USING_IMU = true;
        Globals.USING_IR = true;

        robot = new Robot(getHardwareMap(), telemetry);
        robot.drive.startIMUThread(this);
        robot.sensorArmy.startAllOpMode(this);
    }

    @Override
    public void updateOpMode() {
        if(drive) robot.drive.update();
        if(Faga) robot.faga.periodic();
        if(lift) robot.lift.periodic();
        if(sensorArray) {
            robot.sensorArmy.periodic();
            telemetry.addData("position", robot.sensorArmy.getPosition());
            telemetry.addData("distance", robot.sensorArmy.getDistanceToWall());
        }

        double loop = System.nanoTime();
        hz = 1000000000 / (loop - prevLoopTime);
        prevLoopTime = loop;

        sum += hz;
        count++;

        average = sum/count;

        if(hz > high) high = hz;
        if(hz < low) low = hz;

        telemetry.addData("loop time", loop);
        updateTelemetry(telemetry);
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("hz", hz);
        telemetry.addData("low", low);
        telemetry.addData("high", high);
        telemetry.addData("rolling average", average);

        telemetry.update();
    }
}
