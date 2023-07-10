package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Auto.Paths;
import org.firstinspires.ftc.teamcode.FSM.FiniteStateMachine;
import org.firstinspires.ftc.teamcode.FSM.actions.DropAndReturn;
import org.firstinspires.ftc.teamcode.FSM.actions.FollowTrajectorySequence;
import org.firstinspires.ftc.teamcode.FSM.actions.ParkFSM;
import org.firstinspires.ftc.teamcode.FSM.actions.PickUpCone;
import org.firstinspires.ftc.teamcode.FSM.actions.ToConeStack;
import org.firstinspires.ftc.teamcode.hardware.subsystems.SensorArmy;
import org.firstinspires.ftc.teamcode.opmodes.AutoOpMode;
import org.firstinspires.ftc.teamcode.opmodes.TeleOpMode;
import org.firstinspires.ftc.teamcode.opmodes.comp.auto.AutoConst;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.jetbrains.annotations.NotNull;

@TeleOp(name = "Test FSM's", group = "!")
public class TestFSMOpMode extends AutoOpMode {

    FollowTrajectorySequence dropPreload;
    FollowTrajectorySequence toConeStack;
    ToConeStack tapeAlignCone;
    FollowTrajectorySequence toHighPole;
    ParkFSM park;




    @Override
    public void initOpMode() {
        super.initOpMode();
        dropPreload = new FollowTrajectorySequence(robot, Paths.NormalHigh.Left.dropPreload);
        toConeStack = new FollowTrajectorySequence(robot, Paths.NormalHigh.Left.toConeStackFromPreload, -0.1);
        tapeAlignCone = new ToConeStack(robot, AutoConst.autoTrackingColor, Math.toRadians(180));
        toHighPole = new FollowTrajectorySequence(robot, null); // null will be updated later
        park = new ParkFSM(robot, 0, Paths.NormalHigh.Left.getParkPositions());

    }

    @Override
    public void initLoopOpMode() throws InterruptedException {
        super.initLoopOpMode();
    }

    @Override
    public void updateOpMode() throws InterruptedException {
        super.updateOpMode();
        telemetry.addData("IMU", robot.drive.isImuActive());
        toConeStack.update();
        //dropAndReturn.update();
        //pickUpCone.update();


        //FiniteStateMachine.updateFSMs(pickUpCone);

        if(gamepad1.cross) {
            toConeStack.start();
          //  pickUpCone .start();
        }
        if(gamepad1.circle) {
            toConeStack.stop();
         //   pickUpCone.stop();
        }
        if(gamepad1.dpad_down) {
        //    dropAndReturn.start();
        }

        telemetry.addData("ts", toConeStack.getState());
       // telemetry.addData("pickup", pickUpCone.getState());
       // telemetry.addData("drop", dropAndReturn.getState());
        telemetry.update();
    }
}
