package org.firstinspires.ftc.teamcode.robot.hardware;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * The TerrorPublisher class implements the Publisher-Subscriber model, allowing
 * multiple writing devices to be managed efficiently. The publisher (TerrorPublisher)
 * maintains a list of subscribers (TerrorWritingDevice instances) that perform actions
 * when the publisher's write method is called.
 *
 * <p>In this model:</p>
 * <ul>
 *   <li><strong>Publisher:</strong> TerrorPublisher, which manages writing devices.</li>
 *   <li><strong>Subscriber:</strong> TerrorWritingDevice instances that execute specific actions.</li>
 * </ul>
 */
public class TerrorPublisher {
    private final List<TerrorWritingDevice> writingDevices;

    /**
     * Constructs a new TerrorPublisher instance, initializing the list of writing devices.
     */
    public TerrorPublisher() {
        this.writingDevices = new ArrayList<>();
    }

    /**
     * Subscribes one or more TerrorWritingDevice instances to the publisher.
     *
     * @param devices Varargs of TerrorWritingDevice instances to be added.
     */
    public void subscribe(TerrorWritingDevice... devices) {
        writingDevices.addAll(Arrays.asList(devices));
    }

    /**
     * Writes data to all subscribed writing devices by invoking their write methods.
     */
    public void write() {
        for (TerrorWritingDevice device : writingDevices) {
            device.write();
        }
    }
}
