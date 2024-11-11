package org.firstinspires.ftc.teamcode.robot.init;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.math.Coordinate;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.robot.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.drive.localizer.OTOSLocalizer;
import org.firstinspires.ftc.teamcode.robot.drive.mecanum.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.robot.hardware.sensors.SampleColor;
import org.firstinspires.ftc.teamcode.robot.hardware.sensors.camera.TerrorCamera;
import org.firstinspires.ftc.teamcode.robot.subsystems.InOutTake;
import org.firstinspires.ftc.teamcode.robot.subsystems.PinkArm;

/**
 * A class containing all the robot's subsystems.
 */
@Config
public class Robot {
    public static double OTOS_LINEAR_SCALAR = 1.1371628089414737246168002475814;
    public static double OTOS_ANGULAR_SCALAR = 1.0066;

    // dash
    public static Pose2d OTOS_OFFSET = new Pose2d(0, 3.55, 0);
    public static Pose2d ROBOT_INITIAL_POS = new Pose2d(0, 0, 0);

    // Subsystems
    public Drivetrain drivetrain = null;
    public PinkArm pinkArm = null;
    public InOutTake inOutTake = null;

    // Localizer
    public OTOSLocalizer localizer;

    // Camera stuff
    public TerrorCamera camera;

    // Other misc public objects
    public FtcDashboard dashboard;
    public MultipleTelemetry telemetry;
    public RobotHardware hardware;

    // rando
    private double hangTimer = 0;

    public void init(@NonNull LinearOpMode opMode, @NonNull RobotHardware hardware, Telemetry tele) {
        // Save local copy of RobotHardware class
        this.hardware = hardware;

        // Set up dashboard stuff
        this.dashboard = FtcDashboard.getInstance();
        this.telemetry = new MultipleTelemetry(tele, dashboard.getTelemetry());

        // Set up subsytems
        this.pinkArm = new PinkArm(hardware);
        this.inOutTake = new InOutTake(hardware);

        // Initialize the localizer
        this.localizer = new OTOSLocalizer(hardware.otos);
        localizer.initializeOtos(new OTOSLocalizer.Parameters(
                OTOS_OFFSET,
                ROBOT_INITIAL_POS,
                OTOS_LINEAR_SCALAR,
                OTOS_ANGULAR_SCALAR
        ));

        // Initialize the drivetrain
        this.drivetrain = new MecanumDrivetrain(
                hardware.motorRearLeft,
                hardware.motorFrontLeft,
                hardware.motorRearRight,
                hardware.motorFrontRight
        );
    }

    /**
     * Intake a sample
     * @param colors The colors to intake
     * @param armPosition The position of the arm
     */
    public void intake(SampleColor[] colors, PinkArm.State armPosition) {
        this.inOutTake.intake(colors);
        this.pinkArm.setState(armPosition);
    }

    /**
     * Outtake a sample
     * @param armPosition The position of the arm
     */
    public void outtake(PinkArm.State armPosition) {
        this.inOutTake.outtake();
        this.pinkArm.setState(armPosition);
    }

    /**
     * Move the arm manually
     * @param pitch The amount to add to the pitch, in radians
     * @param extension The amount to add to the extension, in inches
     */
    public void moveArm(double pitch, double extension) {
        this.pinkArm.adjustPitch(pitch);
        this.pinkArm.adjustExtension(extension);
    }
    
    public void rotateArmRaw(double power) {
        this.pinkArm.setPitchPower(power);
    }

    /**
     * Hang the robot, toggling between various stages of hang
     * CALL THIS FUNCTION EVERY LOOP ITERATION
     */
    public void hang() {
        if (!this.pinkArm.isHanging()) {
            this.pinkArm.setState(PinkArm.State.HANG_1);
        } else if (this.pinkArm.atTargetPosition() &&
                this.pinkArm.stateIs(PinkArm.State.HANG_1)) {
            this.hardware.dtPto.setPosition(1 /* idk smth this is placeholder */);

            // set a 1 second timer before going to hang 2
            if (this.hangTimer == 0) {
                this.hangTimer = System.nanoTime();
            } else if (((System.nanoTime() - this.hangTimer) / 1e9) >= 1) {
                this.pinkArm.setState(PinkArm.State.HANG_2);
                this.drivetrain.move(new Coordinate(1, 1), 0);
            }
        }
    }

    public void setRobotInitialPos(Pose2d initialPos){
        this.ROBOT_INITIAL_POS=initialPos;
    }
}
