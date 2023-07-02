package org.firstinspires.ftc.teamcode.FSM.actions;

import org.firstinspires.ftc.teamcode.FSM.FiniteStateMachine;

public class PickUpCone extends FiniteStateMachine {

    public PickUpCone() {
        initStates();
    }

    //Robot robot;

    //public PickUpCone(Robot robot) {
    //    this.robot = robot;
    //    initStates();
    //}

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
                nextState();
                break;
            case "DROPPING_LIFT":
                print("drop lift");
                nextState(stateTimer.seconds() > 0.2);
                break;
            case "GRAB_CONE":
                print("grab cone");
                nextState(stateTimer.seconds() > 0.4);
                break;
            case "LIFT_ARTICULATE":
                print("lift");
                print("articulate");
                nextState();
                break;
            case "END":
                print("ended");
                reset();
                break;
        }
    }
}
