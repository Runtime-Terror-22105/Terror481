package org.firstinspires.ftc.teamcode.robot.hardware.sensors;

import com.qualcomm.robotcore.hardware.AnalogInput;

public class TerrorAnalogEncoder {
    private double offset = 0;
    private AnalogInput encoder = null;

    public TerrorAnalogEncoder(AnalogInput encoder) {
        this.encoder = encoder;
    }

    /**
     * Returns the CURRENT position of the axon servo from the absolute encoder.
     * @return The absolute position.
     */
    public double getCurrentPosition() {
        return (this.encoder.getVoltage() / 3.3) * Math.PI*2 + this.offset;
    }

    /**
     * Sets an offset to be added to the return value of getPosition()
     * @param offset The offset
     */
    public void setOffset(double offset) {
        this.offset = offset;
    }
}
