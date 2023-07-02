package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Eyelids;
import org.firstinspires.ftc.teamcode.hardware.subsystems.SensorArmy;
import org.firstinspires.ftc.teamcode.opmodes.TeleOpMode;

import java.util.Arrays;

@Disabled
@Config
@TeleOp(name = "Sensor Testing", group = "!Tuning")
public class SensorTesting extends TeleOpMode {
    SensorArmy sensorArmy;

    MultipleTelemetry m_telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


    @Override
    public void initOpMode() {
        sensorArmy = new SensorArmy(getHardwareMap());
    }

    @Override
    public void updateOpMode() {
        m_telemetry.addData("zero", 0);


        m_telemetry.addData("Right", sensorArmy.rightDistance.getDistance(DistanceUnit.INCH));
        m_telemetry.addData("Left", sensorArmy.leftDistance.getDistance(DistanceUnit.INCH));
        //m_telemetry.addData("right smooth", sensorArmy.getRightIR());
        //m_telemetry.addData("left smooth", sensorArmy.getLeftIR());


        m_telemetry.addData("l2", sensorArmy.sensorsArray[0].red());
        m_telemetry.addData("l1", sensorArmy.sensorsArray[1].red());
        m_telemetry.addData("m", sensorArmy.sensorsArray[2].red());
        m_telemetry.addData("r1", sensorArmy.sensorsArray[3].red());
        m_telemetry.addData("r2", sensorArmy.sensorsArray[4].red());

        m_telemetry.addData("sensor array", Arrays.toString(sensorArmy.sensorActive));
        //m_telemetry.addData("smooth autograb", sensorArmy.getSmoothedAutoGrab(DistanceUnit.MM));
        //m_telemetry.addData("autograb position", sensorArmy.autoGrab.getDistance(DistanceUnit.MM));
        m_telemetry.update();
    }

    @Override
    public void stopOpMode() {

    }
}
