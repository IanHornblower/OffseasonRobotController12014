package org.firstinspires.ftc.teamcode.opmodes.comp.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auto.Paths;
import org.firstinspires.ftc.teamcode.FSM.FiniteStateMachine;
import org.firstinspires.ftc.teamcode.FSM.actions.FollowTrajectorySequence;
import org.firstinspires.ftc.teamcode.FSM.actions.ParkFSM;
import org.firstinspires.ftc.teamcode.FSM.actions.PickUpCone;
import org.firstinspires.ftc.teamcode.FSM.actions.ToConeStack;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Lift;
import org.firstinspires.ftc.teamcode.opmodes.AutoOpMode;

import static org.firstinspires.ftc.teamcode.Auto.Paths.SingletonActions.*;

@Autonomous(name = "Left High CRI", group = "!")
public class LeftHighCRI extends AutoOpMode {

    FollowTrajectorySequence dropPreload;
    FollowTrajectorySequence toConeStack;
    FollowTrajectorySequence toHighPole;
    ToConeStack tapeAlignCone;
    ParkFSM park;

    PickUpCone pickUpCone;

    boolean parked = false;
    boolean preload = true;

    public static int cycleCount = 4;

    @Override
    public void initOpMode() {
        super.initOpMode();

        robot.drive.setPoseEstimate(Paths.NormalHigh.Left.start);

        dropPreload = new FollowTrajectorySequence(robot, Paths.NormalHigh.Left.dropPreload, -0.1);
        toConeStack = new FollowTrajectorySequence(robot, Paths.NormalHigh.Left.toConeStackFromPreload, -0.1);
        tapeAlignCone = new ToConeStack(robot, AutoConst.autoTrackingColor, Math.toRadians(180));
        toHighPole = new FollowTrajectorySequence(robot, Paths.NormalHigh.Left.toHighPole, -0.1); // null will be updated later
        park = new ParkFSM(robot, 1, Paths.NormalHigh.Left.getParkPositions());
        pickUpCone = new PickUpCone(robot);

        robot.faga.clawClose();
    }

    @Override
    public void initLoopOpMode() throws InterruptedException {
        super.initLoopOpMode();
    }

    @Override
    public void startOpMode() {
        super.startOpMode();

     //   park.setParkPosition(getLocation().getValue());

        dropPreload.start();

        timer.reset();
    }

    @Override
    public void updateOpMode() throws InterruptedException {
        super.updateOpMode();
        //FiniteStateMachine.updateFSMs(Singletons);
        FiniteStateMachine.updateFSMs(dropPreload, toConeStack, tapeAlignCone, toHighPole, park, pickUpCone);

        if(dropPreload.ended() || toHighPole.ended()) {
            robot.faga.clawOpen();
            preload = false;
        }

        toConeStack.start(
                (dropPreload.ended() || toHighPole.ended()) &&
                        cycle < cycleCount
        );

        tapeAlignCone.start(toConeStack.ended());
        if(tapeAlignCone.ended()) {
            if(cycle == 0) {  // only do once after the first alignment
                toConeStack.updateTrajectorySequence(Paths.NormalHigh.Left.toConeStackFromCycle);
            }
            //always update the initial position to avoid jostling after localization
            toHighPole.updateTrajectorySequence(Paths.NormalHigh.Left.getHighPole(tapeAlignCone.getReloclization()));
        }

        //pickUpCone.start(robot.sensorArmy.getAutoGrabDistance() < 55);
        pickUpCone.start(robot.sensorArmy.getDistanceToWall() > 4 && !preload);
        toHighPole.start(tapeAlignCone.ended());
        //toHighPole.start(pickUpCone.ended());
        //if(cycle > cycleCount && park.getState().equals("IDLE") && !parked) {
        //    park.start();
        //}
        park.start(cycle > cycleCount && park.getState().equals("IDLE") && !parked);

        if(park.ended()) parked = true; // if parked once don't do it again

        /*
         * End of main FSM
         */

        if(toHighPole.ended()) cycle++; // increment cycle count

        telemetry.addData("reloc", tapeAlignCone.getReloclization().toString());
        telemetry.addData("Auto Time", timer.seconds());

        telemetry.addData("lift debug", robot.lift.incrmital);
        telemetry.addData("lift target", robot.lift.getTarget());
        telemetry.addData("lift error", robot.lift.getError());
        telemetry.addData("lift state", robot.lift.state.toString());
        telemetry.addData("fourbar state", robot.faga.state.toString());

        //telemetry.addData("drop preload", dropPreload.getState());
        //telemetry.addData("preload to stack", toConeStack.getState());
        telemetry.addData("tape align", tapeAlignCone.getState());
        telemetry.addData("to high pole", toHighPole.getState());
        //telemetry.addData("park state", park.getState());

        telemetry.update();

    }
}
