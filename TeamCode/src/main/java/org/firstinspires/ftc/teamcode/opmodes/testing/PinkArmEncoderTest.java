package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

@Autonomous(group = "testing")
public class PinkArmEncoderTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware hardware = new RobotHardware();
        Robot robot = new Robot();
        hardware.init(hardwareMap, LynxModule.BulkCachingMode.AUTO);
        robot.init(this, hardware, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            double pitch = robot.hardware.armPitchEncoder.getCurrentPosition();
            robot.telemetry.addData("Current Extension (ticks)", robot.pinkArm.getExtensionPosition());
            robot.telemetry.addData("Current Pitch (degrees)", Math.toDegrees(pitch));
            robot.telemetry.addData("Current Pitch (radians)", pitch);
            robot.telemetry.update();
        }
    }
}
