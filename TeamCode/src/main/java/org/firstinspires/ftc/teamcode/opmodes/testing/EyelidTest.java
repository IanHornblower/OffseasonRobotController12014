package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.subsystems.Eyelids;
import org.firstinspires.ftc.teamcode.OldAssShit.TeleOpModeDEPRICATED;

@Disabled
@Config
@TeleOp(name = "Eyelid Testing", group = "Tuning")
public class EyelidTest extends TeleOpModeDEPRICATED {
    Eyelids eyelids;

    public static boolean up = false;

    MultipleTelemetry m_telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


    @Override
    public void initOpMode() {
        eyelids = new Eyelids(getHardwareMap());
    }

    @Override
    public void updateOpMode() {
        if(up) {
            eyelids.up();
        }
        else {
            eyelids.down();
        }

        m_telemetry.addData("is up", up);
        m_telemetry.update();
    }

    @Override
    public void stopOpMode() {

    }
}
