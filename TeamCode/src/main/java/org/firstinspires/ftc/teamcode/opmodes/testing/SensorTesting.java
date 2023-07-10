package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.subsystems.SensorArmy;
import org.firstinspires.ftc.teamcode.OldAssShit.TeleOpModeDEPRICATED;
import org.firstinspires.ftc.teamcode.opmodes.AutoOpMode;

import java.util.Arrays;

@Config
@TeleOp(name = "Sensor Testing", group = "!Tuning")
public class SensorTesting extends AutoOpMode {
    SensorArmy sensorArmy;

    public static boolean blue = false;
    public static boolean ir = false;
    public static double offsetConstant = -0.6; // Primary Guess Value
    private double loopTime = 0;

    MultipleTelemetry m_telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    // if telem is weird only use telem since it's already multi in TeleOpMode framework

    @Override
    public void initOpMode() {
        super.initOpMode();

        sensorArmy = robot.sensorArmy;
    }

    @Override
    public void initLoopOpMode() throws InterruptedException {
        super.initLoopOpMode();
    }


    @Override
    public void updateOpMode() {
        if(ir) {
            m_telemetry.addData("Right", sensorArmy.getLeftIR());
            m_telemetry.addData("Left", sensorArmy.getRightIR());

            m_telemetry.addData("get offset", sensorArmy.getDistanceOffset());
        }

        if(blue) {
            if(sensorArmy.folowingColor.equals(SensorArmy.Color.RED)) sensorArmy.setFollowingColor(SensorArmy.Color.BLUE);

            m_telemetry.addData("l2", sensorArmy.getArray()[0].blue());
            m_telemetry.addData("l1", sensorArmy.getArray()[1].blue());
            m_telemetry.addData("m", sensorArmy.getArray()[2].blue());
            m_telemetry.addData("r1", sensorArmy.getArray()[3].blue());
            m_telemetry.addData("r2", sensorArmy.getArray()[4].blue());
        }
        else {
            if(sensorArmy.folowingColor.equals(SensorArmy.Color.BLUE)) sensorArmy.setFollowingColor(SensorArmy.Color.RED);

            m_telemetry.addData("l2", sensorArmy.getArray()[0].red());
            m_telemetry.addData("l1", sensorArmy.getArray()[1].red());
            m_telemetry.addData("m", sensorArmy.getArray()[2].red());
            m_telemetry.addData("r1", sensorArmy.getArray()[3].red());
            m_telemetry.addData("r2", sensorArmy.getArray()[4].red());
        }

        double offset = offsetConstant * sensorArmy.getPosition(); // maybe rethink a better way to calculate this idk :)

        m_telemetry.addData("raw tape position", sensorArmy.getPosition());
        m_telemetry.addData("real life offset", offset);

        m_telemetry.addData("sensor array", Arrays.toString(sensorArmy.getActive())); // should actually work :O

        double loop = System.nanoTime();
        m_telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        loopTime = loop;
        m_telemetry.update();
    }

    @Override
    public void stopOpMode() {

    }
}
