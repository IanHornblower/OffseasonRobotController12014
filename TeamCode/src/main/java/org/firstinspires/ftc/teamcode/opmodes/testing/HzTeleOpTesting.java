package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.OldAssShit.TeleOpModeDEPRICATED;

@Config
@TeleOp
public class HzTeleOpTesting extends TeleOpModeDEPRICATED {
    double low = 999;
    double high = 0;
    double filtered = 0;
    double average = 0;
    double sum = 0;
    double count = 0;
    double loop = 0;
    double loopTime = 0;

    public static double p_liftPos = 0.0;
    public static double liftPos = 0.0;

    public static double p_4barPos = 0.0;
    public static double fourbarPos = 0.0;

    public static boolean camera = false;
    public static boolean eyelids = false;
    public static boolean Faga = false;
    public static boolean intake = false;
    public static boolean drive = false;
    public static boolean lift = false;
    public static boolean mogus = false;
    public static boolean sensorArray = false;
    public static boolean ENABLE_ROBOT = false;
    public static boolean bulk = false;

    private LynxModule.BulkData data = null;

    double prevLoopTime = 0;
    double hz = 0;

    Robot robot;

    @Override
    public void initOpMode() {
        Globals.USING_TAPE_SENSORS = true;
        Globals.USING_IMU = true;
        Globals.USING_IR = true;

        robot = new Robot(getHardwareMap(), telemetry);
        robot.drive.startIMUThread(this, false);
        robot.sensorArmy.startAllOpMode(this, false);
    }

    @Override
    public void updateOpMode() {
        if(Faga || lift || bulk) {
            data = PhotonCore.EXPANSION_HUB.getBulkData();
        }

        telemetry.addData("IMU", robot.drive.isImuActive());
        telemetry.addData("Tape", robot.sensorArmy.isTapeActive());
        telemetry.addData("IR", robot.sensorArmy.isIrActive());
        telemetry.addData("AutoGrab", robot.sensorArmy.isAutoGrabActive());

        if(drive) robot.drive.update();
        if(Faga) {
            robot.faga.loop(data);
            if(fourbarPos != p_4barPos) {
                robot.faga.setFourbarPosition(fourbarPos);
                p_4barPos = fourbarPos;
            }
        }
        if(lift) {
            robot.lift.loop(data);
            if(liftPos != p_liftPos) {
                robot.lift.setTarget(liftPos);
                p_liftPos = liftPos;
            }
        }

        if(sensorArray) {
            telemetry.addData("position", robot.sensorArmy.getPosition());
            telemetry.addData("distance", robot.sensorArmy.getDistanceToWall());
        }

        loop = System.nanoTime();
        loopTime = (loop - prevLoopTime);
        hz = 1000000000 / loopTime;
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
        telemetry.addData("loop time", loopTime/1e+6);

        telemetry.update();
    }
}
