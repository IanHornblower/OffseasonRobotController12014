package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxUnsupportedCommandException;
import com.qualcomm.hardware.lynx.commands.LynxCommand;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.interfaces.Subsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Camera;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Eyelids;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Faga;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Lift;

import org.firstinspires.ftc.teamcode.hardware.subsystems.SensorArmy;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.PID.PIDController;

public class Robot implements Subsystem {

    public Eyelids eyelids;
    public Faga faga;
    public Intake intake;
    public Lift lift;
    public SensorArmy sensorArmy;
    public SampleMecanumDrive drive;
    public Camera camera;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        eyelids = new Eyelids(hardwareMap);
        faga = new Faga(hardwareMap);
        intake = new Intake(hardwareMap);
        lift = new Lift(hardwareMap);
        sensorArmy = new SensorArmy(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        camera = new Camera(hardwareMap, telemetry);
    }

    @Override
    public void periodic() { //TODO: maybe ion know - To get loop times lower only read BulkData when I need to
                             // request a read for data otherwise dont poll the ExHub.
        LynxModule.BulkData data = PhotonCore.EXPANSION_HUB.getBulkData();

        faga.loop(data);
        lift.loop(data);
    }

    public void updateFSMs() {
        if(Globals.IS_AUTO) {
            //update auto fsm's
        }
        else {
            //update tele fsm's
        }

        //update all other fsm's
    }

    public void stop() {
        //faga.stop();
        lift.stop();
        drive.stop();
        intake.stop();
    }
}
