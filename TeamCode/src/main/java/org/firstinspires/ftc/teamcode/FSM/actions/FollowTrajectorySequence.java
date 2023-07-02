package org.firstinspires.ftc.teamcode.FSM.actions;

import org.firstinspires.ftc.teamcode.FSM.FiniteStateMachine;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class FollowTrajectorySequence extends FiniteStateMachine {

    private final Robot robot;
    private TrajectorySequence trajectorySequence;
    private final double endOffset;

    public FollowTrajectorySequence(Robot robot, TrajectorySequence trajectorySequence) {
        this.robot = robot;
        this.trajectorySequence = trajectorySequence;
        endOffset = 0.0;

        addState("IDLE");
        addState("STARTING");
        addState("RUNNING");
        addState("END");
    }

    public FollowTrajectorySequence(Robot robot, TrajectorySequence trajectorySequence, double endOffset) {
        this.robot = robot;
        this.trajectorySequence = trajectorySequence;
        this.endOffset = endOffset;

        addState("IDLE");
        addState("STARTING");
        addState("RUNNING");
        addState("END");
    }

    public void updateTrajectorySequence(TrajectorySequence trajectorySequence) {
        this.trajectorySequence = trajectorySequence;
    }

    @Override
    public void update() {
        switch(getState()) {
            case "IDLE":
                break;
            case "STARTING":
                robot.drive.followTrajectorySequence(trajectorySequence);
                stateTimer.reset();

                setState("RUNNING");
                break;
            case "RUNNING":
                robot.drive.update();

                nextState(trajectorySequence.duration() + endOffset > stateTimer.seconds());
                break;
            case "END":
                robot.drive.stop();
                reset();
                break;
        }
    }
}
