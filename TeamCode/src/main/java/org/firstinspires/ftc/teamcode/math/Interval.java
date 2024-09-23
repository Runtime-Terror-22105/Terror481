package org.firstinspires.ftc.teamcode.math;

import com.acmerobotics.roadrunner.util.NanoClock;

// A simple class to repeat a certain action after a certain time period
public class Interval {
    private double pollTime;
    private double lastPollTime;
    private final NanoClock timer;

    // pollTime is in milliseconds
    public Interval(double pollTime) {
        this.pollTime = pollTime;
        this.timer = NanoClock.system();
        this.lastPollTime = 0;
    }

    public boolean intervalHasPassed() {
        if (getTimeMillis() >= (lastPollTime + pollTime)) {
            this.lastPollTime = this.getTimeMillis();
            return true;
        }

        return false;
    }

    public double getTimeMillis() {
        return this.timer.seconds()*1e3;
    }

    public double getLastPollTime() {
        return this.lastPollTime;
    }

    public void setPollTime(double pollTime) {
        this.pollTime = pollTime;
    }
}
