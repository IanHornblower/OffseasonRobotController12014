package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.subsystems.Faga;
import org.firstinspires.ftc.teamcode.OldAssShit.TeleOpModeDEPRICATED;

@Config
@TeleOp(name = "Fourbar Tuner", group = "Tuning")
public class FourbarPIDFTuning extends TeleOpModeDEPRICATED {
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

    public static double p_4barPos = 0.0;


    @Override
    public void updateOpMode() {
        faga.loop(PhotonCore.EXPANSION_HUB.getBulkData());

        if(position != p_4barPos) {
            faga.setFourbarPosition(position);
            p_4barPos = position;
        }
        faga.updatePIDF();
        faga.setArticulation(articulationPosition);
        faga.setClawPosition(clawPosition);


        //if(intaked) faga.returnToIntake();
        //else faga.setToOutake();

        //m_telemetry.addData("velocity", faga.getFourbar().getVelocity());
        m_telemetry.addData("state", faga.state.toString());
        m_telemetry.addData("error", faga.getError());
        m_telemetry.addData("zero", 0);
        m_telemetry.addData("fourbar des", faga.getTarget());
        m_telemetry.addData("fourbar enc", faga.getState());
        m_telemetry.addData("Volatage", getHardwareMap().voltageSensor.iterator().next().getVoltage());
        m_telemetry.update();

    }

    @Override
    public void stopOpMode() {
        faga.stopFourbar();
    }
}
