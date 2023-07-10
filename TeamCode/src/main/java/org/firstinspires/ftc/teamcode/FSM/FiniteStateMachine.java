package org.firstinspires.ftc.teamcode.FSM;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

public class FiniteStateMachine {

    private int state = 0;
    private boolean hasEnded = false;
    private ArrayList<String> FSM_STATE = new ArrayList<>();
    public ElapsedTime stateTimer = new ElapsedTime();

    // Used in the creation of a FSM to define what states there are
    public void addState(String str) {
        FSM_STATE.add(str);
    }

    // Return what state the FSM is in
    public String getState() {
        return FSM_STATE.get(state);
    }

    public int getIndex() {
        return state;
    }

    // Augment the current state to 1 to start the FSM
    public void start() {
        hasEnded = false;

        state = 1;
        stateTimer.reset();
    }

    public void start(Boolean bool) {
        if(bool) {
            start();
        }
    }

    public void nextState(double time) {
        nextState(stateTimer.seconds() > time);
    }

    public void nextState() {
        if(state + 1 > FSM_STATE.size() - 1) reset();
        else {
            stateTimer.reset();
            state++;
        }
    }

    public void setState(String str) {
        state = FSM_STATE.indexOf(str);
    }

    public void nextState(boolean bool) {
        if(bool) nextState(); // weird behavior
    }

    // always run // Make sure to override
    public void update() {}

    // When called set to the last state, usually stopping and resetting the FSM
    public void stop() {
        state = FSM_STATE.size() - 1;
    }

    // When called reset the first state
    public void reset() {
        state = 0;
        stateTimer.reset();
    }

    public boolean ended() {
        if(getState().equals("END")) hasEnded = true;
        return getState().equals("END");
    }

    public boolean hasEnded() {
        return hasEnded;
    }

    public boolean idle() {
        return getState().equals("IDLE");
    }

    @NotNull
    public static void updateFSMs(FiniteStateMachine... fsms) {
        for(FiniteStateMachine fsm: fsms) {
            fsm.update();
        }
    }
}
