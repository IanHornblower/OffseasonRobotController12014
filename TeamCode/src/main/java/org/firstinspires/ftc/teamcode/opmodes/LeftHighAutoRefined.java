package org.firstinspires.ftc.teamcode.opmodes;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Auto.Paths;
import org.firstinspires.ftc.teamcode.FSM.FiniteStateMachine;
import org.firstinspires.ftc.teamcode.FSM.actions.FollowTrajectorySequence;
import org.firstinspires.ftc.teamcode.FSM.actions.ParkFSM;
import org.firstinspires.ftc.teamcode.FSM.actions.PickUpCone;
import org.firstinspires.ftc.teamcode.FSM.actions.ToConeStack;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.SensorArmy;

@Disabled
@TeleOp(name = "Left High Auto Refined", group = "!!!!!!")
public class LeftHighAutoRefined extends LinearOpMode {
    @Override
    public void runOpMode() {
        MultipleTelemetry m_telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        Robot robot = new Robot(hardwareMap, telemetry);

        Paths.setDrive(robot);

        robot.drive.setPoseEstimate(Paths.NormalHigh.Left.start);

        robot.drive.startIMUThread(this, false);

        int cycle = 1;
        boolean parked = false;

        FollowTrajectorySequence dropPreload = new FollowTrajectorySequence(robot, Paths.NormalHigh.Left.dropPreload);
        FollowTrajectorySequence toConeStack = new FollowTrajectorySequence(robot, Paths.NormalHigh.Left.toConeStackFromPreload, -0.1);
        ToConeStack tapeAlignCone = new ToConeStack(robot, SensorArmy.Color.BLUE, Math.toRadians(180));
        PickUpCone pickUpCone = null;
        FollowTrajectorySequence toHighPole = new FollowTrajectorySequence(robot, null); // null will be updated later
        ParkFSM park = new ParkFSM(robot, 0, Paths.NormalHigh.Left.getParkPositions());

        waitForStart();

        park.setParkPosition(2);

        dropPreload.start(); // Start the auto sequence

        while (opModeIsActive() && !isStopRequested()) {
            FiniteStateMachine.updateFSMs( // update loop for fsm's
                    dropPreload,
                    toConeStack,
                    tapeAlignCone,
                    toHighPole,
                    pickUpCone,
                    park
            );

            /* TODO: add two distance sensors for angle correction on reloc
             *  Add Color Tracking Switching for this opmode
             *  Add actual parks for this
             *  Add File for all the Trajectory Sequence Segments (RR0.5.6 actions)
             *  Create PRIVATE github for code with MTI code and then CRI code bases for parity
             *  Multithread IMU on AS project
             *  Find Everywhere I can speed up i2c read speeds, as well as reduce loop times
             *
             * Auto FSM Flow:
             *
             * preload
             * -- Loop start --
             * to cone stack
             * align with tape
             * to high pole
             * -- Loop to start -- / Break cycle after the 4th cycle
             * park
             *
             */

            toConeStack.start(
                    (dropPreload.ended() || toHighPole.ended()) &&
                            cycle < 5
            );
            tapeAlignCone.start(toConeStack.ended());
            if(tapeAlignCone.ended()) {
                if(cycle == 1) {  // only do once after the first alignment
                    toConeStack.updateTrajectorySequence(Paths.NormalHigh.Left.toConeStackFromCycle);
                }
                //always update the initial position to avoid jostling after localization
                toHighPole.updateTrajectorySequence(Paths.NormalHigh.Left.getHighPole(tapeAlignCone.getReloclization()));
            }

            pickUpCone.start(tapeAlignCone.ended());
            toHighPole.start(tapeAlignCone.ended());
            park.start(cycle > 5 && park.getState().equals("IDLE") && !parked);

            if(park.ended()) parked = true; // if parked once don't do it again

            /*
             * End of main FSM
             */

            if(toHighPole.ended()) cycle++; // increment cycle count

            m_telemetry.addData("park state", park.getState());
            m_telemetry.addData("reloc", tapeAlignCone.getReloclization().toString());
            telemetry(robot, m_telemetry);
        }
    }

    public void telemetry(Robot robot, Telemetry telemetry) {
        telemetry.addData("to wall", robot.sensorArmy.getDistanceToWall());

        telemetry.addData("pose", robot.drive.getPoseEstimate().toString());
        telemetry.addData("position", robot.sensorArmy.getPosition());



        double offset = -0.55 * robot.sensorArmy.getPosition();
        offset = -0.6 * robot.sensorArmy.getPosition();

        telemetry.addData("corrected position", -12 - offset);

        telemetry.update();
    }
}