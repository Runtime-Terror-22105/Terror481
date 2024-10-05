package org.firstinspires.ftc.teamcode.robot.hardware.motors;

import androidx.annotation.NonNull;

import com.outoftheboxrobotics.photoncore.hardware.servo.PhotonCRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * A wrapper CRServo class for axon servos that has two goals.
 * 1) Provide caching to avoid unnecessary setPower() lynxcommands.
 * 2) Allow for easy usage of the Axon servos.
 */
@Deprecated
public class TerrorAxonServo {
    private double offset = 0;
    private double lastPower;
    private AnalogInput absoluteServoEncoder = null;
    private final PhotonCRServo crservo;

    private final double powerThreshold;

    public TerrorAxonServo(@NonNull PhotonCRServo crservo, double powerThreshold) {
        this.powerThreshold = powerThreshold;
        this.crservo = crservo;
        this.lastPower = crservo.getPower();
    }

    /**
     * Sets the absolute servo encoder.
     * @param absoluteServoEncoder The analog port of the servo encoder.
     */
    public void setAbsoluteServoEncoder(AnalogInput absoluteServoEncoder) {
        this.absoluteServoEncoder = absoluteServoEncoder;
    }

    /**
     * Sets power to the CRServo.
     * @param power The power to set, between -1 and 1.
     */
    synchronized public void setPower(double power) {
        if (Math.abs(this.lastPower - power) > this.powerThreshold) {
            this.lastPower = power;
            this.crservo.setPower(power);
        }
    }

    /**
     * Sets the direction of the servo.
     * @param direction The direction of the servo.
     */
    public void setDirection(DcMotorSimple.Direction direction) {
        this.crservo.setDirection(direction);
    }

    /**
     * Returns the CURRENT position of the axon servo from the absolute encoder.
     * @return The absolute position.
     */
    public double getAbsolutePosition() {
        return this.absoluteServoEncoder.getVoltage() / 3.3 * Math.PI*2 + this.offset;
    }

    /**
     * Sets an offset to be added to the return value of getPosition()
     * @param offset The offset
     */
    public void setOffset(double offset) {
        this.offset = offset;
    }
}
