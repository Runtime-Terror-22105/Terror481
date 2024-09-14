package org.firstinspires.ftc.teamcode.robot.drive.mecanum;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.hardwareDevices.TerrorMotor;
import org.firstinspires.ftc.teamcode.math.Coordinate;

public class MecanumDrivetrain implements Drivetrain {
    private final Telemetry telemetry;
    private final TerrorMotor motorBackLeft;
    private final TerrorMotor motorFrontLeft;
    private final TerrorMotor motorBackRight;
    private final TerrorMotor motorFrontRight;

    /**
     * Initializes a swerve drivetrain.
     * @param telemetry The telemetry
     * @param motorBackLeft Self explanatory
     * @param motorFrontLeft Self explanatory
     * @param motorBackRight Self explanatory
     * @param motorFrontRight Self explanatory
     */
    public MecanumDrivetrain(Telemetry telemetry, TerrorMotor motorBackLeft, TerrorMotor motorFrontLeft,
                             TerrorMotor motorBackRight, TerrorMotor motorFrontRight) {
        this.telemetry = telemetry;
        this.motorBackLeft = motorBackLeft;
        this.motorFrontLeft = motorFrontLeft;
        this.motorBackRight = motorBackRight;
        this.motorFrontRight = motorFrontRight;

        this.motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Move the robot by some amount
     * @param velocity Movement on x and y
     * @param rotation Rotation
     */
    @Override
    public void move(@NonNull Coordinate velocity, double rotation) {
        this.motorBackLeft.setPower(velocity.y - velocity.x + rotation);
        this.motorFrontLeft.setPower(velocity.y + velocity.x + rotation);
        this.motorBackRight.setPower(velocity.y + velocity.x - rotation);
        this.motorFrontRight.setPower(velocity.y - velocity.x - rotation);
    }
}
