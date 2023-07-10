package org.firstinspires.ftc.teamcode.FSM.actions;

import org.firstinspires.ftc.teamcode.FSM.FiniteStateMachine;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Lift;

public class DropAndReturn extends FiniteStateMachine {
    Robot robot;

    public DropAndReturn(Robot robot) {
        this.robot = robot;
        initStates();
    }

    public void initStates() {
        addState("IDLE");
        addState("STARTING");
        addState("DROP");
        addState("RETURN");
        addState("REBOUND");
        addState("RETURN_FORBAR");
        addState("END");
    }

    @Override
    public void update() {
        switch(getState()) {
            case "IDLE":
                break;
            case "STARTING":
                setState("DROP");
                break;
            case "DROP":
                robot.faga.clawOpen();
                nextState(0.2);
                break;
            case "RETURN":
                robot.faga.clawTransfer();
                robot.lift.setTarget(Lift.LIFT.RETURN);
                robot.faga.articulateIntake();
                robot.faga.setFourbarPosition(0);
                nextState(Math.abs(robot.lift.getError()) < 30); // wierd behavior
                break;
            case "REBOUND":
                robot.faga.clawOpen();
                robot.lift.setTarget(Lift.LIFT.INTAKE);
                nextState(); // wierd behavior
                break;
            case "RETURN_FORBAR":
                robot.faga.setManuelPower(-0.8);
                nextState(Math.abs(robot.faga.getError()) < 20);
                break;
            case "END":
                robot.faga.setManuelPower(0.0);
                reset();
                break;
        }
    }
}
