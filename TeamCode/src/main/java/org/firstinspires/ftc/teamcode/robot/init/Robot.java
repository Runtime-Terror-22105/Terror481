package org.firstinspires.ftc.teamcode.robot.init;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.math.Pose2d;
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
    // otos dash vals
    public static double OTOS_LINEAR_SCALAR = 1.1371628089414737246168002475814;
    public static double OTOS_ANGULAR_SCALAR = 1.0066;
    public static Pose2d OTOS_OFFSET = new Pose2d(0, 3.55, 0);
    public static Pose2d ROBOT_INITIAL_POS = new Pose2d(0, 0, 0);

    // States
    public RobotState robotState = RobotState.NONE;

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

    public void setRobotInitialPos(Pose2d initialPos){
        this.ROBOT_INITIAL_POS=initialPos;
    }

    /**
     * Change the robot's state if it is not in the correct state.
     * @param desiredState The desired state of the robot.
     * @return Whether or not the state was changed. If it is already in that state, returns false.
     */
    public boolean setState(RobotState desiredState) {
        if (robotState.equals(desiredState)) {
            return false;
        }
        robotState = desiredState;

        switch (robotState) {
            case RESTING:
                goToRestingState();
            case INTAKE:
                goToIntakeState();
            case BUCKET:
                goToBucketState();
            case SPECIMEN:
                goToSpecimenState();
        }

        pinkArm.update();


        return true;
    }

    private void goToRestingState() {
        // move pink arm (pitch and extension 0)
        pinkArm.resetPitch();
        pinkArm.resetExtension();
        // stuff with the rollers
    }

    private void goToIntakeState() {
        // move pink arm (arm pitched down, slightly extended ~5-10 inches)
        pinkArm.resetPitch();
        pinkArm.setExtensionTarget(40);
        // stuff with the rollers
    }

    private void goToBucketState() {
        // move pink arm (arm pitched up ~90-95 deg, max extension ~40-50 in)
        pinkArm.setPitchTarget(Math.PI/2);
        pinkArm.setExtensionTarget(450);
        // stuff with the rollers
    }

    private void goToSpecimenState() {
        // move pink arm (arm slightly pitched ~45 deg, partial extension ~5-10 in)
        pinkArm.setPitchTarget(Math.PI/4);
        pinkArm.setExtensionTarget(40);
        // stuff with the rollers
    }
}
