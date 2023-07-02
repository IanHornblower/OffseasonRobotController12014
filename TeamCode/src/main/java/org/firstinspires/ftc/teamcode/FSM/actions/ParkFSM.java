package org.firstinspires.ftc.teamcode.FSM.actions;


import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class ParkFSM extends FollowTrajectorySequence {
    TrajectorySequence[] tsa;

    public ParkFSM(Robot robot, int sleevePosition, TrajectorySequence... trajectorySequence) {
        super(robot, trajectorySequence[0]);

        tsa = trajectorySequence;

        updateTrajectorySequence(trajectorySequence[sleevePosition]);

    }

    public void setParkPosition(int location) {
        updateTrajectorySequence(tsa[location]);
    }

    @Override
    public void update() {
        super.update();
    }
}
