package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Faga;
import org.firstinspires.ftc.teamcode.opmodes.TeleOpMode;

@Config
@TeleOp(name = "Fourbar Tuner", group = "Tuning")
public class FourbarPIDFTuning extends TeleOpMode {
    Faga faga;

    MultipleTelemetry m_telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    public static double position = 0;
    public static double articulationPosition = 0.5;
    public static double clawPosition = 0.5;

    public static boolean intaked = true;

    @Override
    public void initOpMode() {
        faga = new Faga(getHardwareMap());
        faga.resetFourbarEncoder();
    }

    @Override
    public void updateOpMode() {
        faga.setFourbarPosition(position);
        faga.updatePIDF();
        faga.setArticulation(articulationPosition);

        //if(intaked) faga.returnToIntake();
        //else faga.setToOutake();

        faga.updateFourbar();

        m_telemetry.addData("mp target", faga.mp_target);
        m_telemetry.addData("velocity", faga.getFourbar().getVelocity());
        m_telemetry.addData("zero", 0);
        m_telemetry.addData("fourbar des", position);
        m_telemetry.addData("fourbar enc", faga.getFourbar().getCurrentPosition());
        m_telemetry.addData("Volatage", getHardwareMap().voltageSensor.iterator().next().getVoltage());
        m_telemetry.update();

        faga.setClawPosition(clawPosition);
    }

    @Override
    public void stopOpMode() {
        faga.stopFourbar();
    }
}
