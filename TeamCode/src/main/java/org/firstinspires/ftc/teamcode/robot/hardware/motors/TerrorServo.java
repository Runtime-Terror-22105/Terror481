package org.firstinspires.ftc.teamcode.robot.hardware.motors;

import androidx.annotation.NonNull;

import com.outoftheboxrobotics.photoncore.hardware.servo.PhotonServo;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * A wrapper servo class that provides caching to avoid unnecessary setPosition() calls.
 */
public class TerrorServo {
    private final PhotonServo servo;
    private double lastPosition;

    private final double positionThreshold = 0.0000;

    public TerrorServo(@NonNull PhotonServo servo) {
        this.servo = servo;
        this.lastPosition = servo.getPosition();
    }

    synchronized public void setPosition(double position) {
        if (Math.abs(this.lastPosition - position) < this.positionThreshold) {
            this.lastPosition = position;
            this.servo.setPosition(position);
        }
    }

    public void setDirection(Servo.Direction direction) {
        this.servo.setDirection(direction);
    }
}
