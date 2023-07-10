package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;

import org.firstinspires.ftc.teamcode.OldAssShit.TeleOpModeDEPRICATED;

@Disabled
@Config
@TeleOp(name = "Intake Test", group = "Tuning")
public class DriveTrainIntakeTest extends TeleOpModeDEPRICATED {
    Intake intake;

    public static double intakePower = 0.0;

    MultipleTelemetry m_telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


    @Override
    public void initOpMode() {
        intake = new Intake(hardwareMap);
    }

    @Override
    public void updateOpMode() {
        intake.setPower(intakePower);

        m_telemetry.addData("power", intakePower);
        m_telemetry.update();
    }

    @Override
    public void stopOpMode() {
    }
}
