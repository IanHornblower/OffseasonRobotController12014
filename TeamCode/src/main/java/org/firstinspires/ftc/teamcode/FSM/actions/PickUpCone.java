package org.firstinspires.ftc.teamcode.FSM.actions;

import org.firstinspires.ftc.teamcode.FSM.FiniteStateMachine;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.RobotConstants;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Lift;

import java.util.PriorityQueue;

public class PickUpCone extends FiniteStateMachine {
    Robot robot;

    public PickUpCone(Robot robot) {
        this.robot = robot;
        initStates();
    }

    public void initStates() {
        addState("IDLE");
        addState("STARTING");
        addState("DROPPING_LIFT");
        addState("GRAB_CONE");
        addState("LIFT_ARTICULATE");
        addState("END");
    }

    public void print(Object obj) {
        System.out.println(obj);
    }

    @Override
    public void update() {
        switch(getState()) {
            case "IDLE":
                break;
            case "STARTING":
                robot.faga.setManuelPower(-0.65);
                setState("DROPPING_LIFT");
                break;
            case "DROPPING_LIFT":
                robot.lift.setPower(-0.55);

                if(Globals.IS_AUTO) nextState(0.2);
                else nextState(robot.lift.getState() < 15);
                break;
            case "GRAB_CONE":
                robot.faga.setManuelPower(0);
                robot.faga.clawClose();
                nextState(0.2);
                break;
            case "LIFT_ARTICULATE":
                if(Globals.IS_AUTO) {
                    robot.faga.setArticulation(RobotConstants.Faga.Articulator.autoStack);
                    robot.lift.setTarget(500);
                     robot.lift.target = 500;
                    robot.lift.state = Lift.STATE.RUNNING;
                }
                else {
                    robot.faga.setFourbarPosition(3200);
                    robot.faga.articulateIntake();
                }
                nextState();
                break;
            case "END":
                //print("ended");
                reset();
                break;
        }
    }
}
