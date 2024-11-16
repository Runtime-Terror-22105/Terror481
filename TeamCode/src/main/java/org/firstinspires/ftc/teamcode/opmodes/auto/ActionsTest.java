package org.firstinspires.ftc.teamcode.opmodes.auto;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.robot.auto.followers.P2PFollower;
import org.firstinspires.ftc.teamcode.robot.auto.followers.Task;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

@Config
@TeleOp
public class ActionsTest extends LinearOpMode {
    private RobotHardware hardware = new RobotHardware();
    private final Robot robot = new Robot();
    private FtcDashboard dashboard;
    @Override
    public void runOpMode() throws InterruptedException {
        hardware.init(hardwareMap, LynxModule.BulkCachingMode.AUTO);
        robot.init(this, hardware, telemetry);

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        waitForStart();

        P2PFollower follower = new P2PFollower.Builder(robot)
                .executeActionOnce("Telemetry adding", (Task.Context ctx) -> {
                    robot.telemetry.addData("typeskib", "omg");
                    robot.telemetry.update();
                }, 100)
                // point, tolerance xyh
                .addPoint(
                        new Pose2d(10, 10, Math.PI/2),
                        new Pose2d(1, 1, Math.toRadians(2)),
                        0.2,
                        5000
                )
                // pass in any function
                .executeUntilTrue(
                        "Get current time until x <= 2",
                        (Task.Context ctx) -> ctx.getCurrentPos().x <= 2,
                        (Task.Context ctx) -> {
                            robot.telemetry.addData("Current time", System.currentTimeMillis());
                            robot.telemetry.update();
                        },
                        10000
                )
                // NOTE: there is no finishActions() here, so we go to the next point while this action is still ongoing
                .addPoint(
                        new Pose2d(0, 0, 0),
                        new Pose2d(1, 1, 0.1),
                        0.2,
                        5000
                )
                .build();

        follower.follow(this::opModeIsActive, robot.localizer::getPosition);
    }
}
