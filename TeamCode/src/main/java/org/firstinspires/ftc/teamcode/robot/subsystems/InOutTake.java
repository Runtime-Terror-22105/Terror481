package org.firstinspires.ftc.teamcode.robot.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.robot.hardware.motors.TerrorCRServo;
import org.firstinspires.ftc.teamcode.robot.hardware.motors.TerrorServo;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

@Config
public class InOutTake {
    public static double SERVO_1_UP = 1.0;
    public static double SERVO_1_DOWN = 0;
    public static double SERVO_2_UP = 1.0;
    public static double SERVO_2_DOWN = 0;

    private final TerrorCRServo wheelServo1;
    private final TerrorCRServo wheelServo2;
    private final TerrorServo pitchServo;

    public InOutTake(@NonNull RobotHardware hardware) {
        this.wheelServo1 = hardware.intakeWheelServo1;
        this.wheelServo2 = hardware.intakeWheelServo2;
        this.pitchServo = hardware.intakePitchServo;
    }

    public void intake() {
        wheelServo1.setPower(1.0);
        wheelServo2.setPower(1.0);
    }

    public void outtake() {
        wheelServo1.setPower(-1.0);
        wheelServo2.setPower(-1.0);
    }

    public void stop() {
        wheelServo1.setPower(0);
        wheelServo2.setPower(0);
    }

    public void moveUp() {
        pitchServo.setPosition(SERVO_1_UP);
    }

    public void moveDown() {
        pitchServo.setPosition(SERVO_1_DOWN);
    }
}
