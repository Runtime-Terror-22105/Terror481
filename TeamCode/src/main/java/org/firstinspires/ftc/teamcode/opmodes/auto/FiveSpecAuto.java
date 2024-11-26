package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.robot.auto.followers.P2PFollower;
import org.firstinspires.ftc.teamcode.robot.auto.followers.Task;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

@Autonomous(name = "5 Specimen Auto", group = "Auto")
public class FiveSpecAuto extends LinearOpMode {
    private RobotHardware hardware = new RobotHardware();
    private final Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        hardware.init(hardwareMap, LynxModule.BulkCachingMode.AUTO);
        robot.init(this, hardware, telemetry);

        waitForStart();

        P2PFollower follower = new P2PFollower.Builder(robot)
                .executeUntilTrue(
                        "Pitch arm for depositing",
                        (Task.Context ctx) -> robot.pinkArm.atTargetPosition(),
                        (Task.Context ctx) -> {
                            robot.pinkArm.setPitchTarget(Math.PI/4);
                            robot.pinkArm.update();
                        },
                        2000
                )
                .addPoint(
                        "Go to deposit position",
                        new Pose2d(0,10,0),
                        new Pose2d(0.2, 0.2, Math.toRadians(2)),
                        0.3,
                        2000
                )

                .executeUntilTrue(
                        "Pitch arm back down",
                        (Task.Context ctx) -> robot.pinkArm.atTargetPosition(),
                        (Task.Context ctx) -> {
                            robot.pinkArm.setPitchTarget(0);
                            robot.pinkArm.update();
                        },
                        2000
                )
                .addPoint(
                        "Go to deposit position",
                        new Pose2d(0,10,0),
                        new Pose2d(0.2, 0.2, Math.toRadians(2)),
                        0.3,
                        2000
                )

                .addPoint(
                        "Grab first sample",
                        new Pose2d(12,20,-1.021),
                        new Pose2d(0.2, 0.2, Math.toRadians(2)),
                        0.3,
                        2000
                )
                .addPoint(
                        "Give to human player first sample",
                        new Pose2d(12,20,-2.0082),
                        new Pose2d(0.2, 0.2, Math.toRadians(2)),
                        0.3,
                        2000
                )
                .addPoint(
                        "Grab second sample",
                        new Pose2d(25,26,-1.188),
                        new Pose2d(0.2, 0.2, Math.toRadians(2)),
                        0.3,
                        2000
                )
                .addPoint(
                        "Give to human player second sample",
                        new Pose2d(25,26,-2.266),
                        new Pose2d(0.2, 0.2, Math.toRadians(2)),
                        0.3,
                        2000
                )
                .addPoint(
                        "Grab third sample",
                        new Pose2d(32.3,27.95,-1.13066),
                        new Pose2d(0.2, 0.2, Math.toRadians(2)),
                        0.3,
                        2000
                )
                .addPoint(
                        "Give to human player third sample",
                        new Pose2d(25,26,-2.5493),
                        new Pose2d(0.2, 0.2, Math.toRadians(2)),
                        0.3,
                        2000
                )

                .sleep(this::sleep, 1000L)

                .executeUntilTrue(
                        "Pitch arm down",
                        (Task.Context ctx) -> robot.pinkArm.atTargetPosition(),
                        (Task.Context ctx) -> {
                            robot.pinkArm.setPitchTarget(Math.toRadians(0));
                            robot.pinkArm.update();
                        },
                        10000
                )
                .addPoint(
                        "Intake first specimen",
                        new Pose2d(14.7, 21.1, -2.4285),
                        new Pose2d(0.2, 0.2, Math.toRadians(2)),
                        0.3,
                        2000
                )
                .executeUntilTrue(
                        "Pitch arm for deposit",
                        (Task.Context ctx) -> robot.pinkArm.atTargetPosition(),
                        (Task.Context ctx) -> {
                            robot.pinkArm.setPitchTarget(Math.toRadians(87));
                            robot.pinkArm.update();
                        },
                        10000
                )
                .addPoint(
                        "Deposit first specimen",
                        new Pose2d(-5.45, 27.36, Math.PI),
                        new Pose2d(0.2, 0.2, Math.toRadians(2)),
                        0.3,
                        2000
                )

                .executeUntilTrue(
                        "Pitch arm back down",
                        (Task.Context ctx) -> robot.pinkArm.atTargetPosition(),
                        (Task.Context ctx) -> {
                            robot.pinkArm.setPitchTarget(0);
                            robot.pinkArm.update();
                        },
                        10000
                )
                .addPoint(
                        "Intake second specimen",
                        new Pose2d(14.7, 21.1, -2.4285),
                        new Pose2d(0.2, 0.2, Math.toRadians(2)),
                        0.3,
                        2000
                )
                .executeUntilTrue(
                        "Pitch arm for deposit",
                        (Task.Context ctx) -> robot.pinkArm.atTargetPosition(),
                        (Task.Context ctx) -> {
                            robot.pinkArm.setPitchTarget(Math.toRadians(87));
                            robot.pinkArm.update();
                        },
                        10000
                )
                .addPoint(
                        "Deposit second specimen",
                        new Pose2d(-3.45, 27.36, Math.PI),
                        new Pose2d(0.2, 0.2, Math.toRadians(2)),
                        0.3,
                        2000
                )

                .executeUntilTrue(
                        "Pitch arm back down",
                        (Task.Context ctx) -> robot.pinkArm.atTargetPosition(),
                        (Task.Context ctx) -> {
                            robot.pinkArm.setPitchTarget(0);
                            robot.pinkArm.update();
                        },
                        10000
                )
                .addPoint(
                        "Intake third specimen",
                        new Pose2d(14.7, 21.1, -2.4285),
                        new Pose2d(0.2, 0.2, Math.toRadians(2)),
                        0.3,
                        2000
                )
                .executeUntilTrue(
                        "Pitch arm for deposit",
                        (Task.Context ctx) -> robot.pinkArm.atTargetPosition(),
                        (Task.Context ctx) -> {
                            robot.pinkArm.setPitchTarget(Math.toRadians(87));
                            robot.pinkArm.update();
                        },
                        10000
                )
                .addPoint(
                        "Deposit third specimen",
                        new Pose2d(-2.45, 27.36, Math.PI),
                        new Pose2d(0.2, 0.2, Math.toRadians(2)),
                        0.3,
                        2000
                )

                .executeUntilTrue(
                        "Pitch arm back down",
                        (Task.Context ctx) -> robot.pinkArm.atTargetPosition(),
                        (Task.Context ctx) -> {
                            robot.pinkArm.setPitchTarget(0);
                            robot.pinkArm.update();
                        },
                        10000
                )
                .addPoint(
                        "Intake fourth specimen",
                        new Pose2d(14.7, 21.1, -2.4285),
                        new Pose2d(0.2, 0.2, Math.toRadians(2)),
                        0.3,
                        2000
                )
                .executeUntilTrue(
                        "Pitch arm for deposit",
                        (Task.Context ctx) -> robot.pinkArm.atTargetPosition(),
                        (Task.Context ctx) -> {
                            robot.pinkArm.setPitchTarget(Math.toRadians(87));
                            robot.pinkArm.update();
                        },
                        10000
                )
                .addPoint(
                        "Deposit fourth specimen",
                        new Pose2d(-1.45, 27.36, Math.PI),
                        new Pose2d(0.2, 0.2, Math.toRadians(2)),
                        0.3,
                        2000
                )

                .executeUntilTrue(
                        "Pitch arm back down",
                        (Task.Context ctx) -> robot.pinkArm.atTargetPosition(),
                        (Task.Context ctx) -> {
                            robot.pinkArm.setPitchTarget(0);
                            robot.pinkArm.update();
                        },
                        10000
                )
                .addPoint(
                        "Skibidi Park",
                        new Pose2d(34.2, 3.6, Math.PI),
                        new Pose2d(0.2, 0.2, Math.toRadians(2)),
                        0.3,
                        2000
                )
                .build();

        follower.follow(this::opModeIsActive, robot.localizer::getPosition);
    }
}
