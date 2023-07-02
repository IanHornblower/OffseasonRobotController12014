package org.firstinspires.ftc.teamcode.hardware.subsystems;

import org.firstinspires.ftc.teamcode.hardware.interfaces.Subsystem;
import org.firstinspires.ftc.teamcode.util.OpModeInformations;
import org.firstinspires.ftc.teamcode.util.Timer;

public class Mogus implements Subsystem {

    int index, interval;
    Timer timer = new Timer();

    public Mogus() {
        interval = 80;

        index = 0;
        timer.start();
    }

    public Mogus(int interval) {
        this.interval = interval;

        index = 0;
        timer.start();
    }

    public void setInterval(int interval) {
        this.interval = interval;
    }

    public String get35pxTwerk() {
        if(index > 4) index = 0;
        return OpModeInformations.amoungUsTwerk35px[index];
    }

    public String get38pxTwerk() {
        if(index > 4) index = 0;
        return OpModeInformations.amoungUsTwerk38px[index];
    }

    @Override
    public void periodic()  {
        if(timer.currentMills() > 50) {
            index++;
            timer.reset();
        }
    }

}
