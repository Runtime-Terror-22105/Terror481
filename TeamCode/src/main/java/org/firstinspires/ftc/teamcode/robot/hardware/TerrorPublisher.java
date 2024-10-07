package org.firstinspires.ftc.teamcode.robot.hardware;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class TerrorPublisher {
    private final List<TerrorWritingDevice> writingDevices;

    public TerrorPublisher() {
        this.writingDevices = new ArrayList<>();
    }

    public void subscribe(TerrorWritingDevice... devices) {
        writingDevices.addAll(Arrays.asList(devices));
    }

    public void write() {
        for (TerrorWritingDevice device : writingDevices) {
            device.write();
        }
    }
}
