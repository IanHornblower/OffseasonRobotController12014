package org.firstinspires.ftc.teamcode.opmodes.comp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.RobotConstants;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Lift;
import org.firstinspires.ftc.teamcode.opmodes.TeleOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "#!main")
public class TeleOp extends TeleOpMode {
    Robot robot;

    boolean eyelidToggle = false;
    boolean inCorrect = false;
    double speed = 0.6;
    ElapsedTime eyelidTimer;
    ElapsedTime grabConeTimer;
    ElapsedTime toggleDropTimer;
    double loopTime = 0;

    boolean toggleDrop = false;

    GamepadEx operator;

    @Override
    public void initOpMode() {
        robot = new Robot(getHardwareMap(), telemetry);
        eyelidTimer = new ElapsedTime();
        grabConeTimer = new ElapsedTime();
        toggleDropTimer = new ElapsedTime();

        operator = new GamepadEx(gamepad2);
    }

    @Override
    public void initLoopOpMode() throws InterruptedException {
        robot.init();
        //robot.faga.resetFourbarEncoder();
        robot.returnCone();


        //PhotonCore.enable();
    }

    @Override
    public void startOpMode() {
        robot.returnCone();
    }

    @Override
    public void updateOpMode() throws InterruptedException {
        robot.periodic();

        operator.readButtons();

        robot.drive.setDrivePower(new Pose2d(gamepad1.left_stick_y * speed, gamepad1.left_stick_x * speed, -gamepad1.right_stick_x* 0.8));

        if((gamepad1.right_trigger > 0.1) && !(gamepad1.left_trigger > 0.1)) {
            robot.intake.start();
        }
        else if(!(gamepad1.right_trigger > 0.1) && (gamepad1.left_trigger > 0.1)) {
            robot.intake.reverse();
        }
        else {
            robot.intake.stop();
        }

        if(gamepad1.left_bumper) {
            speed = 0.6;
        }
        if(gamepad1.right_bumper) {
            speed = 0.95;
        }

        /*
         * EYELIDS Controls
         */

        if(gamepad1.cross && eyelidTimer.seconds() > 0.3 && eyelidToggle) {
            robot.eyelids.up();
            eyelidToggle = !eyelidToggle;
            eyelidTimer.reset();
            inCorrect = false;
        }
        else if(gamepad1.cross && eyelidTimer.seconds() > 0.3 && !eyelidToggle) {
            robot.eyelids.down();
            eyelidToggle = !eyelidToggle;
            eyelidTimer.reset();
            inCorrect = false;
        }

        if(gamepad1.circle && eyelidTimer.seconds() > 0.3 && !inCorrect) {
            robot.eyelids.correct();
            eyelidTimer.reset();
            eyelidToggle = true;
            inCorrect = true;
        }
        else if((gamepad1.circle || gamepad2.circle) && eyelidTimer.seconds() > 0.3 && inCorrect) {
            robot.eyelids.up();
            inCorrect = false;
            eyelidTimer.reset();
        }
        if(gamepad2.square) {
            robot.lift.setTarget(Lift.LIFT.LOW_FRONTLOAD);
            robot.faga.setFourbarPosition(2300);
            robot.faga.setArticulation(0.54);
        }
        if(gamepad2.left_trigger > 0.1) {
            robot.faga.clawOpen();
        }

        /*
         * END of Driver Controls
         */

        /*
         * Operator Controls
         */

        if(gamepad2.right_trigger > 0.1 && robot.sensorArmy.getSmoothedAutoGrab(DistanceUnit.MM) < 35 && robot.grabConeState == Robot.grabCone.IDLE && grabConeTimer.seconds() > 1) {
            robot.grabCone();
            grabConeTimer.reset();
        }

        if(gamepad2.right_bumper) {
            robot.grabConeTele();
        }

        //if(gamepad2.left_bumper) {
        if(operator.isDown(GamepadKeys.Button.LEFT_BUMPER) && !(gamepad2.left_trigger > 0.1) && !toggleDrop) {
            toggleDrop = true;

            //robot.returnCone();
        }

        if ((operator.stateJustChanged(GamepadKeys.Button.LEFT_BUMPER) && !(gamepad2.left_trigger > 0.1)) && operator.wasJustReleased(GamepadKeys.Button.LEFT_BUMPER)) {
            robot.returnCone(80);

            toggleDrop = false;
        }

        if(gamepad2.left_trigger > 0.1 && gamepad2.left_bumper && toggleDropTimer.seconds() > 1) {
            robot.faga.articulateOuttake();
            robot.faga.setFourbarPosition(robot.faga.getFourbar().getCurrentPosition() - 250);

            toggleDropTimer.reset();
            toggleDrop = false;
        }


        if(gamepad2.dpad_left) {
            robot.lift.setTarget(Lift.LIFT.LOW);
            robot.faga.setToOutake();
        }

        if(gamepad2.dpad_up) {
            robot.lift.setTarget(Lift.LIFT.MID);
            robot.faga.setToOutake();
        }

        if(gamepad2.dpad_right) {
            robot.lift.setTarget(Lift.LIFT.HIGH);
            robot.faga.setToOutake();
        }

        if(gamepad2.dpad_down) {
            robot.lift.setTarget(Lift.LIFT.RETURN);
            robot.faga.setFourbarPosition(100);
            robot.faga.setArticulation(RobotConstants.Faga.Articulator.intake);
        }

        if(gamepad2.triangle) {
            robot.faga.setFourbarPosition(robot.faga.getFourbar().getCurrentPosition() + 200);
        }
        if(gamepad2.cross) {
            robot.faga.setFourbarPosition(robot.faga.getFourbar().getCurrentPosition() - 100);
        }

        if(gamepad2.left_stick_button) {
            robot.drive.relax();
        }
        if(gamepad2.right_stick_button) {
            robot.drive.retract();
        }

        // Manual Lift
        robot.lift.setManuelPower(-Math.pow(gamepad2.left_stick_y,3));

        double loop = System.nanoTime();
        telemetry.addData("Speed", speed);
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        telemetry.update();
        loopTime = loop;


    }

    @Override
    public void stopOpMode() {
        robot.drive.stop();
        robot.faga.getFourbar().setPower(0.0);
    }
}
