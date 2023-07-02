package org.firstinspires.ftc.teamcode.util.FSM.test;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.hardware.subsystems.Faga;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Lift;
import org.firstinspires.ftc.teamcode.util.FSM.FiniteStateMachine;

@Config
public class DropAndReturnCone extends FiniteStateMachine {

    Lift lift;
    Faga faga;

    public static double resting = 80;
    public static double openToReturnDelay = 0.15;

    public DropAndReturnCone(Lift lift, Faga faga) {
        this.lift = lift;
        this.faga = faga;

        addState("IDLE");
        addState("DROP");
        addState("RETURN");
        addState("END");
    }

    @Override
    public void update() {
        switch (getState()) {
            case "IDLE":
                break;
            case "DROP":
                faga.clawOpen();

                if(stateTimer.seconds() > openToReturnDelay) nextState();
                break;
            case "RETURN":
                faga.clawTransfer();
                lift.setPosition(resting);
                faga.returnToIntake();

                if(faga.getFourbar().getCurrentPosition() < 2200) nextState();
                break;
            case "END":
                faga.clawOpen();

                nextState();
                break;
        }
    }
}
