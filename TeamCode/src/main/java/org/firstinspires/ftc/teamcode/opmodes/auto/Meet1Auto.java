package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.math.Coordinate;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.init.RobotState;

@Autonomous
public class Meet1Auto extends LinearOpMode {
    private RobotHardware hardware = new RobotHardware();
    private final Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        hardware.init(hardwareMap, LynxModule.BulkCachingMode.AUTO);
        robot.init(this, hardware, telemetry);

        waitForStart();

//        PidToPoint p2p = new PidToPoint(
//                new Pose2d(0, -12, 0),
//                new Pose2d(1, 1, 1),
//                0.3
//        );
//        while (opModeIsActive()) {
//            Pose2d currentPos = robot.localizer.getPosition();
//            Pose2d powers = p2p.calculatePower(currentPos);
//            boolean reached = p2p.driveToDestination(robot.drivetrain, powers, currentPos);
//
//            if (reached) {
//                break;
//            }
//            hardware.write();
//        }

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.milliseconds() < 1.5 * 1000) {
            robot.drivetrain.move(new Coordinate(0, -0.4), 0);
            hardware.write();
        }

        robot.setState(RobotState.BUCKET);
        while (opModeIsActive() && !robot.pinkArm.atTargetPosition()) {
            robot.pinkArm.update();
            hardware.write();
        }

        robot.inOutTake.outtake();
        while (opModeIsActive() && timer.milliseconds() < 1.5 * 1000) {
            hardware.write();
        }

        robot.setState(RobotState.RESTING);
        while (opModeIsActive() && !robot.pinkArm.atTargetPosition()) {
            robot.pinkArm.update();
            hardware.write();
        }
    }
}
