package org.firstinspires.ftc.teamcode.opmodes.teleop;

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
        ElapsedTime drivingTimer = new ElapsedTime();
        Gamepad lastGamepad1 = gamepad1;
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

//            if (gamepad1.a && !lastGamepad1.a) { // rising edge
//                robot.inOutTake.intake(new SampleColor[] {
//                        SampleColor.YELLOW,
//                        SampleColor.BLUE
//                });
//            } else if (gamepad1.x && !lastGamepad1.x) { // rising edge
//                robot.inOutTake.outtake();
//            }
//
//            if (gamepad1.y && !lastGamepad1.y) {
//                robot.pinkArm.setExtension(10);
//                robot.pinkArm.setPitch(Math.PI/2);
//            } else if (gamepad1.b && !lastGamepad1.b) {
//                robot.pinkArm.setExtension(100);
//                robot.pinkArm.setPitch(0);
//            }

//            robot.inOutTake.update();
//            robot.pinkArm.update();
            lastGamepad1.copy(gamepad1);

            hardware.write();

            robot.telemetry.addData("Heading Angle",headingLockAngle);
            robot.telemetry.addData("Rotation power", rotation);
            robot.telemetry.addData("Loop time (ms)", loopTimer.milliseconds());
            robot.telemetry.addData("Loop time (hz)", 1000/loopTimer.milliseconds());

            robot.telemetry.addData("x power", direction.x);
            robot.telemetry.addData("y power", direction.y);
            robot.telemetry.addData("h power", rotation);
            robot.telemetry.update();
            loopTimer.reset();
        }
    }

    public double slr(double joystick_value){
        return Math.pow((Math.pow(joystick_value,3)+joystick_value)/2, 5);
    }
}
