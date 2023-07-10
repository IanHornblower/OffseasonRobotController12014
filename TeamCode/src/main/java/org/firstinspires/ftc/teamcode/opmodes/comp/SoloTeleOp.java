package org.firstinspires.ftc.teamcode.opmodes.comp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FSM.FiniteStateMachine;
import org.firstinspires.ftc.teamcode.FSM.actions.DropAndReturn;
import org.firstinspires.ftc.teamcode.FSM.actions.PickUpCone;
import org.firstinspires.ftc.teamcode.hardware.RobotConstants;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Lift;
import org.firstinspires.ftc.teamcode.opmodes.TeleOpMode;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "#!main")
public class SoloTeleOp extends TeleOpMode {

    boolean eyelidToggle = false;
    boolean inCorrect = false;
    double speed = 0.6;
    ElapsedTime eyelidTimer;
    ElapsedTime toggleDropTimer;
    ElapsedTime grabConeTimer;
    boolean toggleDrop = false;

    DropAndReturn dropAndReturn;
    PickUpCone pickUpCone;

    @Override
    public void initOpMode() {
        super.initOpMode();

        pickUpCone = new PickUpCone(robot);
        dropAndReturn = new DropAndReturn(robot);

        eyelidTimer = new ElapsedTime();
        toggleDropTimer = new ElapsedTime();
        grabConeTimer = new ElapsedTime();
    }

    @Override
    public void startOpMode() {
        robot.lift.setTarget(Lift.LIFT.INTAKE);
        robot.faga.articulateIntake();

            eyelidTimer.reset();
        toggleDropTimer.reset();
          grabConeTimer.reset();
    }

    @Override
    public void updateOpMode() throws InterruptedException {
        super.updateOpMode();

        FiniteStateMachine.updateFSMs(pickUpCone, dropAndReturn);

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

        /*
         * END of Driver Controls
         */

        /*
         * Operator Controls
         */


        if(gamepad2.square) {
            robot.faga.setFourbarPosition(0);
            robot.lift.setTarget(Lift.LIFT.LOW_FRONTLOAD);
            robot.faga.setArticulation(RobotConstants.Faga.Articulator.frontLoad);
        }
        if(gamepad2.left_trigger > 0.1) {
            robot.faga.clawOpen();
        }

         //Auto grab
        if(gamepad2.right_trigger > 0.1 && robot.sensorArmy.getAutoGrabDistance() < 35 && !pickUpCone.idle() && grabConeTimer.seconds() > 1) {
            pickUpCone.start();
            grabConeTimer.reset();
        }

        // Grab Cone
        if(gamepad2.right_bumper) {
           pickUpCone.start();
        }

        if (gamepad2.left_trigger > 0.1) {
            dropAndReturn.start();

            toggleDrop = false;
        }

        if(gamepad2.dpad_left) {
            robot.lift.setTarget(Lift.LIFT.LOW);
            robot.faga.articulateOuttake();
            robot.faga.setFourbarPosition(RobotConstants.Faga.Fourbar.outtake);
        }

        if(gamepad2.dpad_up) {
            robot.lift.setTarget(Lift.LIFT.MID);
            robot.faga.articulateOuttake();
            robot.faga.setFourbarPosition(RobotConstants.Faga.Fourbar.outtake);
        }

        if(gamepad2.dpad_right) {
            robot.lift.setTarget(Lift.LIFT.HIGH);
            robot.faga.articulateOuttake();
            robot.faga.setFourbarPosition(RobotConstants.Faga.Fourbar.outtake);
        }

        if(gamepad2.dpad_down) {
            robot.lift.setTarget(Lift.LIFT.RETURN);
            robot.faga.setFourbarPosition(100);
            robot.faga.setArticulation(RobotConstants.Faga.Articulator.intake);
        }

        if(gamepad2.left_stick_button) {
            robot.drive.relax();
        }
        if(gamepad2.right_stick_button) {
            robot.drive.retract();
        }

        // Manual Lift
        robot.lift.setManuelPower(-Math.pow(gamepad2.left_stick_y,1));
        robot.faga.setManuelPower(-Math.pow(gamepad2.right_stick_y,1));
        telemetry.addData("Fourbar enc", robot.faga.getState());
        telemetry.addData("Drop Cone State", dropAndReturn.getState());
        telemetry.addData("Pickup", pickUpCone.getState());
        telemetry.addData("Lift State", robot.lift.state.toString());
        telemetry.addData("Fourbar State", robot.faga.state.toString());
        telemetry.update();
    }

    @Override
    public void stopOpMode() {
        robot.drive.stop();
        robot.faga.getFourbar().setPower(0.0);
    }
}
