package org.firstinspires.ftc.teamcode.FSM.actions;

import org.firstinspires.ftc.teamcode.FSM.FiniteStateMachine;

import java.time.OffsetDateTime;

public class Delay extends FiniteStateMachine {
    Runnable runnable;

    public Delay(Runnable runnable) {
        this.runnable = runnable;

        addState("IDLE");
        addState("EXECUTE");
        addState("END");
    }

    boolean started = false;

    public void start(boolean active, double seconds) {
        if(!started) {
            stateTimer.reset();
            started = true;
        }
        if(stateTimer.seconds() > seconds && active) start();
    }

    @Override
    public void update() {
        switch (getState()) {
            case "IDLE":
                break;
            case "EXECUTE":
                runnable.run();
                nextState();
                break;
            case "END":
                reset();
                break;
        }
    }
}
