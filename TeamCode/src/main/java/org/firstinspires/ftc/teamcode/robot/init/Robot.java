package org.firstinspires.ftc.teamcode.robot.init;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.drive.localizer.OTOSLocalizer;
import org.firstinspires.ftc.teamcode.robot.drive.mecanum.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.robot.hardware.sensors.camera.TerrorCamera;
import org.firstinspires.ftc.teamcode.robot.subsystems.InOutTake;
import org.firstinspires.ftc.teamcode.robot.subsystems.PinkArm;

/**
 * A class containing all the robot's subsystems.
 */
@Config
public class Robot {
    // Subsystems
    public Drivetrain drivetrain = null;
//    public PinkArm pinkArm = null;
//    public InOutTake inOutTake = null;

    // Localizer
//    public OTOSLocalizer localizer;

    // Camera stuff
    public TerrorCamera camera;

    // Other misc public objects
    public FtcDashboard dashboard;
    public MultipleTelemetry telemetry;
    public RobotHardware hardware;

    public void init(@NonNull LinearOpMode opMode, @NonNull RobotHardware hardware, Telemetry tele) {
        // Save local copy of RobotHardware class
        this.hardware = hardware;

        // Set up dashboard stuff
        this.dashboard = FtcDashboard.getInstance();
        this.telemetry = new MultipleTelemetry(tele, dashboard.getTelemetry());

        // Set up subsytems
//        this.pinkArm = new PinkArm(hardware);
//        this.inOutTake = new InOutTake(hardware);
//
//        // Initialize the localizer
//        this.localizer = new OTOSLocalizer(hardware.otos);
//        localizer.initializeOtos(new OTOSLocalizer.Parameters(
//                new SparkFunOTOS.Pose2D(0, 0, 0),
//                new SparkFunOTOS.Pose2D(0, 0, 0),
//                1.0,
//                1.0
//        ));

        // Initialize the drivetrain
        this.drivetrain = new MecanumDrivetrain(
                hardware.motorRearLeft,
                hardware.motorFrontLeft,
                hardware.motorRearRight,
                hardware.motorFrontRight
        );

        //
//        leftOdometryPod.setEncoder(this.hardware.odoLeftEncoder);
//        rightOdometryPod.setEncoder(this.hardware.odoRightEncoder);
//        backOdometryPod.setEncoder(this.hardware.odoBackEncoder);
    }
}
