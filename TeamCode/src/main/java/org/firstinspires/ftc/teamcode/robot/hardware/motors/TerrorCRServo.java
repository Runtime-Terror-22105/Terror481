package org.firstinspires.ftc.teamcode.robot.hardware.motors;

import androidx.annotation.NonNull;

import com.outoftheboxrobotics.photoncore.hardware.servo.PhotonCRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.robot.hardware.TerrorHardwareDevice;

/**
 * A wrapper servo class that provides caching to avoid unnecessary setPosition() calls.
 */
public class TerrorCRServo implements TerrorHardwareDevice {
    private final PhotonCRServo crservo;

    private final double powerThreshold;

    private double servoPower;
    private CRServoCommand command = CRServoCommand.NONE;
    private enum CRServoCommand {
        SET_POWER,
        NONE
    }

    public TerrorCRServo(@NonNull PhotonCRServo crservo, double powerThreshold) {
        this.powerThreshold = powerThreshold;
        this.crservo = crservo;
        this.servoPower = crservo.getPower();
    }

    public void setPower(double power) {
        if (Math.abs(this.servoPower - power) > this.powerThreshold) {
            this.servoPower = power;
            this.command = CRServoCommand.SET_POWER;
        }
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        this.crservo.setDirection(direction);
    }

    public void write() {
        if (command.equals(CRServoCommand.SET_POWER)) {
            this.crservo.setPower(servoPower);
        }
        this.command = CRServoCommand.NONE;
    }

}
