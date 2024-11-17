package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.robot.init.RobotState.BUCKET;
import static org.firstinspires.ftc.teamcode.robot.init.RobotState.INTAKE;
import static org.firstinspires.ftc.teamcode.robot.init.RobotState.RESTING;
import static org.firstinspires.ftc.teamcode.robot.init.RobotState.SPECIMEN;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.math.Coordinate;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

@Config
@TeleOp
public class RobotCentricTeleOp extends LinearOpMode {
    public static final double DRIVESPEED = 1.0;
    public static final double EXTENSION_MULTIPLIER = 1.0;
    public static final double PITCH_MULTIPLIER = 0.005;

    private RobotHardware hardware = new RobotHardware();
    private final Robot robot = new Robot();

//    public static PidfController.PidfCoefficients heading_coeff = new PidfController.PidfCoefficients(0.63, 0, 0.0624, 0, 0.04);
//
//    public static PidfController heading = new PidfController(heading_coeff);
    public static double ROTATION_SPEED = 0.003;
    private double headingLockAngle = 0; // the angle to lock the heading to, in radians

    @Override
    public void runOpMode() {
        hardware.init(hardwareMap, LynxModule.BulkCachingMode.MANUAL);
        robot.init(this, hardware, telemetry);

        waitForStart();

        ElapsedTime loopTimer = new ElapsedTime();
        Gamepad lastGamepad1 = new Gamepad();
        Gamepad lastGamepad2 = new Gamepad();
        while (opModeIsActive()) {
            // Manually clear the bulk read cache. Deleting this would be catastrophic b/c stale
            // vals would be used.
            for (LynxModule hub : hardware.allHubs) {
                hub.clearBulkCache();
            }

            Coordinate direction = new Coordinate(slr(gamepad1.left_stick_x), slr(-gamepad1.left_stick_y));
            robot.telemetry.addData("x", direction.x);
            robot.telemetry.addData("y", direction.y);
            double rotation = slr(-gamepad1.right_stick_x);
            robot.drivetrain.move(
                    direction,
                    rotation,
                    DRIVESPEED
            );

            if (gamepad2.a) {
                telemetry.addData("resting state", "");
                robot.setState(RESTING);
            } else if (gamepad2.x) {
                telemetry.addData("specimen state", "");
                robot.setState(SPECIMEN);
            } else if (gamepad2.y) {
                telemetry.addData("bucket state", "");
                robot.setState(BUCKET);
            } else if (gamepad2.b) {
                telemetry.addData("intake state", "");
                robot.setState(INTAKE);
            }

            double extensionChange = -gamepad2.left_stick_y * EXTENSION_MULTIPLIER;
            double pitchChange = -gamepad2.right_stick_y * PITCH_MULTIPLIER;
            robot.pinkArm.adjustExtension(extensionChange);
            robot.pinkArm.adjustPitch(pitchChange);

//            if (gamepad1.right_trigger > 0.2) {
//                robot.inOutTake.intake();
//            } else if (gamepad1.left_trigger > 0.2) {
//                robot.inOutTake.outtake();
//            } else {
//                robot.inOutTake.stop();
//            }

//            if ((gamepad2.left_bumper && gamepad2.right_bumper)
//                    && !(lastGamepad2.left_bumper && lastGamepad2.right_bumper)) {
//                hardware.dtPtoLeft.setPosition(1.0);
//                hardware.dtPtoRight.setPosition(1.0);
//            }

            lastGamepad1.copy(gamepad1);
            lastGamepad2.copy(gamepad2);

            robot.pinkArm.update();
            hardware.write();

            robot.telemetry.addData("Current Extension", robot.pinkArm.getExtensionPosition());
            robot.telemetry.addData("Desired Extension", robot.pinkArm.getExtensionTarget());

            robot.telemetry.addData("Current Pitch (degrees)", robot.pinkArm.getPitchPosition());
            robot.telemetry.addData("Desired Pitch (degrees)", Math.toDegrees(robot.pinkArm.getPitchTarget()));

            robot.telemetry.addData("Current state", robot.robotState.name());

//            robot.telemetry.addData("Heading Angle", headingLockAngle);
//            robot.telemetry.addData("Rotation power", rotation);
            robot.telemetry.addData("Loop time (ms)", loopTimer.milliseconds());
            robot.telemetry.addData("Loop time (hz)", 1000/loopTimer.milliseconds());
//
//            robot.telemetry.addData("x power", direction.x);
//            robot.telemetry.addData("y power", direction.y);
//            robot.telemetry.addData("h power", rotation);

            robot.telemetry.update();

            loopTimer.reset();
        }
    }

    public double slr(double joystick_value){
        return Math.pow((Math.pow(joystick_value,3)+joystick_value)/2, 5);
    }
}
