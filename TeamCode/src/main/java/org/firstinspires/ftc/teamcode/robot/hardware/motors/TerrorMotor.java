package org.firstinspires.ftc.teamcode.robot.hardware.motors;

import androidx.annotation.NonNull;

import com.outoftheboxrobotics.photoncore.hardware.motor.PhotonDcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


public class TerrorMotor {
    private boolean motorEnabled;
    private double lastPower;
    private final PhotonDcMotor motor;

    private final double powerThreshold;

    public TerrorMotor(@NonNull PhotonDcMotor motor, double powerThreshold) {
        this.powerThreshold = powerThreshold;
        this.motor = motor;
        setMotorEnable();
        this.lastPower = motor.getPower();
    }

    public void setMotorEnable() {
        if (!motorEnabled) {
            motorEnabled = true;
            this.motor.setMotorEnable();
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        this.motor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void setMode(DcMotor.RunMode mode) {
        this.motor.setMode(mode);
    }

    public void setMotorDisable()
    {
        if (motorEnabled) {
            motorEnabled = false;
            this.motor.setMotorDisable();
        }
    }

    public boolean isMotorEnabled()
    {
//        return super.isMotorEnabled();
        return motorEnabled; // avoid a hardware call
    }

    public synchronized void setVelocity(double angularRate)
    {
        this.motor.setVelocity(angularRate);
    }

    public synchronized void setVelocity(double angularRate, AngleUnit unit)
    {
        this.motor.setVelocity(angularRate, unit);
    }

    public void setPower(double power) {
        if (Math.abs(this.lastPower - power) > this.powerThreshold) {
            this.lastPower = power;
            this.motor.setPower(power);
        }
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        this.motor.setDirection(direction);
    }

    public DcMotorSimple.Direction getDirection() {
        return this.motor.getDirection();
    }

    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    public double getCurrent(CurrentUnit unit) {
        return motor.getCurrent(unit);
    }

    public double getVelocity() { return this.motor.getVelocity();}

    public DcMotor.RunMode getMode() {
        return this.motor.getMode();
    }
}
