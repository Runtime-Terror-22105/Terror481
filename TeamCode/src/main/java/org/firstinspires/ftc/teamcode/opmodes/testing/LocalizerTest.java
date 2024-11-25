package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

@Config
@TeleOp(group = "testing")
public class LocalizerTest extends LinearOpMode {
    public static final double DRIVESPEED = 1.0;
    private final RobotHardware hardware = new RobotHardware();
    private final Robot robot = new Robot();

    @Override
    public void runOpMode() {
        hardware.init(hardwareMap, LynxModule.BulkCachingMode.OFF);
        robot.init(this, hardware, telemetry);

        waitForStart();

        ElapsedTime loopTimer = new ElapsedTime();
        while (opModeIsActive()) {
//            Coordinate direction = new Coordinate(slr(-gamepad1.left_stick_x), slr(gamepad1.left_stick_y));
//
//            double rotation;
//            rotation = Math.pow(slr(-gamepad1.right_stick_x), 11);
//
//            robot.drivetrain.move(
//                    direction,
//                    rotation,
//                    DRIVESPEED
//            );
//            hardware.write();

            Pose2d curPos = robot.localizer.getPosition();
            robot.telemetry.addData("Current x", curPos.x);
            robot.telemetry.addData("Current y", curPos.y);
            robot.telemetry.addData("Current h", curPos.heading);
            robot.telemetry.addData("Linear scalar", robot.localizer.getLinearScalar());
            robot.telemetry.addData("Angular scalar", robot.localizer.getAngularScalar());
            robot.telemetry.addData("Loop time (ms)", loopTimer.milliseconds());
            robot.telemetry.addData("Loop time (hz)", 1000/loopTimer.milliseconds());
            robot.telemetry.update();
            loopTimer.reset();
        }
    }
    public double slr(double joystick_value){
        return (Math.pow(joystick_value,3)+joystick_value)/2;
    }
}
