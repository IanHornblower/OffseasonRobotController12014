package org.firstinspires.ftc.teamcode.roadrunner.util;

public class Delay {

    private long initialTime;
    boolean initialTimeRead = false;
    boolean hasRun = false;
    boolean onceActive = false;

    double offset;
    Runnable instantRunnable;

    public Delay(double offset, Runnable instantRunnable) {
        this.offset = offset;
        this.instantRunnable = instantRunnable;
    }

    public void onTick(boolean active) {
        if(active) onceActive = true;

        if(!initialTimeRead && onceActive) {
            initialTime = System.nanoTime();
            initialTimeRead = true;
        }

        if(!hasRun) {

            if(nanoToSec(System.nanoTime() - initialTime) > offset && onceActive) {
                instantRunnable.run();
                hasRun = true;
            }
        }
    }

    public double nanoToSec(long nano) {
        return nano / 1e+9;
    }

    public long secToNano(double sec) {
        return (long) (sec * 1e+9);
    }
}
