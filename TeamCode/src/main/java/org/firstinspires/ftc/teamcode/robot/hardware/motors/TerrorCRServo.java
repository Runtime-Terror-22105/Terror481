package org.firstinspires.ftc.teamcode.robot.hardware.motors;

import androidx.annotation.NonNull;

import com.outoftheboxrobotics.photoncore.hardware.servo.PhotonCRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * A wrapper servo class that provides caching to avoid unnecessary setPosition() calls.
 */
public class TerrorCRServo {
    private double lastPower;
    private final PhotonCRServo crservo;

    private final double powerThreshold;

    public TerrorCRServo(@NonNull PhotonCRServo crservo, double powerThreshold) {
        this.powerThreshold = powerThreshold;
        this.crservo = crservo;
        this.lastPower = crservo.getPower();
    }

    public void setPower(double power) {
        if (Math.abs(this.lastPower - power) > this.powerThreshold) {
            this.lastPower = power;
            this.crservo.setPower(power);
        }
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        this.crservo.setDirection(direction);
    }

}
