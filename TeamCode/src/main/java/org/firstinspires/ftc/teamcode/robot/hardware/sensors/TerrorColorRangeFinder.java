package org.firstinspires.ftc.teamcode.robot.hardware.sensors;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DigitalChannel;

public class TerrorColorRangeFinder {
    DigitalChannel pin0;
    DigitalChannel pin1;

    public TerrorColorRangeFinder(@NonNull DigitalChannel pin0, @NonNull DigitalChannel pin1) {
        this.pin0 = pin0;
        this.pin1 = pin1;
    }

    public SampleColor getColor() {
        boolean pin0state = pin0.getState();
        boolean pin1state = pin1.getState();

        if (pin0state && pin1state) {
            return SampleColor.YELLOW;
        } else if (pin0state) {
            return SampleColor.RED;
        } else if (pin1state) {
            return SampleColor.BLUE;
        } else {
            return SampleColor.NONE;
        }
    }

    public boolean matchesColor(@NonNull SampleColor[] colors) {
        SampleColor currentColor = this.getColor();
        for (SampleColor color : colors) {
            if (color.equals(currentColor)) {
                return true;
            }
        }
        return false;
    }
}

