package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Faga;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Lift;
import org.firstinspires.ftc.teamcode.opmodes.TeleOpMode;

@Config
@TeleOp(name = "Lift Tuner", group = "Tuning")
public class LiftTuning extends TeleOpMode {
    Lift lift;

    MultipleTelemetry m_telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    public static double position = 0;
    public static double liftPower = 0;

    public static int mult = 1;

    public static boolean automatic = false;

    @Override
    public void initOpMode() {
        lift = new Lift(getHardwareMap());
        //if(automatic) lift.setModeAutomatic();
        //else lift.setModeManuel();

    }

    double previous = 0;

    @Override
    public void updateOpMode() {
        if(!automatic) liftPower = 0.0;

        lift.loop(PhotonCore.EXPANSION_HUB.getBulkData());

        lift.setManuelPower(-Math.pow(gamepad1.left_stick_y,3));

        if(previous != position) {
            lift.setTarget(position);
            previous = position;
        }



        m_telemetry.addData("error", lift.getError());
        m_telemetry.addData("target", lift.getTarget());
        m_telemetry.addData("gamepad1 leftsticky", gamepad1.left_stick_y);
        m_telemetry.addData("lift position", lift.getState());
        m_telemetry.addData("position", position);
        m_telemetry.update();
    }

    @Override
    public void stopOpMode() {
        lift.setPower(0);
    }
}
