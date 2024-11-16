package org.firstinspires.ftc.teamcode.robot.hardware.sensors;

import com.qualcomm.robotcore.hardware.AnalogInput;

public class TerrorAnalogEncoder {
    private double offset = 0;
    private final AnalogInput encoder;
    private final boolean reversed;

    public TerrorAnalogEncoder(AnalogInput encoder, boolean reversed) {
        this.encoder = encoder;
        this.reversed = reversed;
    }

    /**
     * Returns the CURRENT position of the servo from the absolute encoder.
     * @return The absolute position.
     */
    public double getCurrentPosition() {
        double pos = ((this.encoder.getVoltage() / 1.0) * Math.PI*2 + this.offset) % (2*Math.PI);
        if (reversed) {
            pos = 2*Math.PI - pos;
        }
        return pos;
    }

    /**
     * Returns the raw voltage the port is giving. Only for debugging.
     * @return The voltage read at the analog port.
     */
    public double getVoltage() {
        return this.encoder.getVoltage();
    }

    /**
     * Sets an offset to be added to the return value of getPosition()
     * @param offset The offset
     */
    public void setOffset(double offset) {
        this.offset = offset;
    }
}
