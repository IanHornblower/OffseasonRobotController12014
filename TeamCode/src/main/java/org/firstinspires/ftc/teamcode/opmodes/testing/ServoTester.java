package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.subsystems.Eyelids;
import org.firstinspires.ftc.teamcode.opmodes.TeleOpMode;

@Config
@TeleOp(name ="Servo Tester", group = "Tuning")
public class ServoTester extends TeleOpMode {
    public static double position = 0.5;

    MultipleTelemetry m_telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    Servo servo;

    @Override
    public void initOpMode() {
        servo = getHardwareMap().get(Servo.class, "claw");
    }

    @Override
    public void updateOpMode() {
        servo.setPosition(position);

        m_telemetry.addData("servo position", position);
        m_telemetry.update();
    }

    @Override
    public void stopOpMode() {

    }
}
