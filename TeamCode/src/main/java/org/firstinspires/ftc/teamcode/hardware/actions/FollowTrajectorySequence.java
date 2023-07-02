package org.firstinspires.ftc.teamcode.hardware.actions;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.FSM.FiniteStateMachine;

public class FollowTrajectorySequence extends FiniteStateMachine {

    private final Robot robot;
    private final TrajectorySequence trajectorySequence;
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

    @Override
    public void update() {
        switch(getState()) {
            case "IDLE":
                break;
            case "STARTING":
                robot.drive.followTrajectorySequenceAsync(trajectorySequence);
                nextState();
                break;
            case "RUNNING":
                robot.drive.update();
                nextState(trajectorySequence.duration() + endOffset > stateTimer.seconds());
                break;
            case "END":
                robot.drive.stop();
                nextState();
                break;
        }
    }
}
