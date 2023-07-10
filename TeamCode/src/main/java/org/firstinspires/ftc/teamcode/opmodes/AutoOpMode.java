package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Auto.Paths;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Camera;
import org.firstinspires.ftc.teamcode.opmodes.comp.auto.AutoConst;

public class AutoOpMode extends JWOpMode {

    Camera.State location = Camera.State.LEFT;
    public Pose2d start = new Pose2d();
    public int cycle = 0;

    public void setStart(Pose2d start) {
        this.start = start;
    }

    public int getCycle() {
        return cycle;
    }

    public void setCycle(int cycle) {
        this.cycle = cycle;
    }

    public void incrimentCycle() {
        cycle++;
    }

    public Camera.State getLocation() {
        return location;
    }

    @Override
    public void initOpMode() {
        Globals.IS_AUTO = true;
        Globals.USING_IR = true;
        Globals.USING_IMU = true;
        Globals.USING_TAPE_SENSORS = true;
        Globals.USING_AUTO_GRAB = true; // Maybe if I use this for autograb

        super.initOpMode();
        robot.camera.init();
        robot.faga.resetFourbarEncoder();
        robot.faga.clawClose();

        robot.drive.setPoseEstimate(start);
        Paths.setDrive(robot);

        robot.sensorArmy.setFollowingColor(AutoConst.autoTrackingColor);
        timer.reset();
    }

    @Override
    public void initLoopOpMode() throws InterruptedException {
        super.initLoopOpMode();

        if(!robot.drive.isImuActive()) robot.drive.startIMUThread(this, true);
        if(!robot.sensorArmy.isTapeActive()) robot.sensorArmy.startAllOpMode(this, true);

        robot.camera.runInInit();
        location = robot.camera.getSleeveLocation();

        telemetry.addData("Following Color", robot.sensorArmy.folowingColor.toString());
        telemetry.addData("Sleeve Location", robot.camera.getSleeveLocation().toString());
        telemetry.addData("IMU", robot.drive.isImuActive());
        telemetry.addData("Tape", robot.sensorArmy.isTapeActive());
        telemetry.addData("IR", robot.sensorArmy.isIrActive());
        telemetry.addData("AutoGrab", robot.sensorArmy.isAutoGrabActive());
        telemetry.update();
    }

    @Override
    public void startOpMode() {
        robot.camera.shutdown();
        timer.reset();
    }
}
