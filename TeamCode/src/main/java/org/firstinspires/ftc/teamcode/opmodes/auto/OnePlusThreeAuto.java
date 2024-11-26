package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.robot.auto.followers.P2PFollower;
import org.firstinspires.ftc.teamcode.robot.auto.followers.Task;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

@Autonomous(name = "1+3 Auto", group = "Auto")
public class OnePlusThreeAuto extends LinearOpMode {
    private final RobotHardware hardware = new RobotHardware();
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
                        "Pick up first sample",
                        new Pose2d(-40.0573, 27.0693, 0),
                        new Pose2d(0.2, 0.2, Math.toRadians(2)),
                        0.3,
                        2000
                )
                .executeUntilTrue(
                        "Pitch arm up",
                        (Task.Context ctx) -> robot.pinkArm.atTargetPosition(),
                        (Task.Context ctx) -> {
                            robot.pinkArm.setPitchTarget(Math.toRadians(95));
                            robot.pinkArm.update();
                        },
                        10000
                )
                .addPoint(
                        "Basket position",
                        new Pose2d(-47.7438, 9.2995, -0.8258),
                        new Pose2d(0.2, 0.2, Math.toRadians(2)),
                        0.3,
                        2000
                )

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
                        "Pick up second sample",
                        new Pose2d(-51, 27.0693, 0),
                        new Pose2d(0.2, 0.2, Math.toRadians(2)),
                        0.3,
                        2000
                )
                .executeUntilTrue(
                        "Pitch arm up",
                        (Task.Context ctx) -> robot.pinkArm.atTargetPosition(),
                        (Task.Context ctx) -> {
                            robot.pinkArm.setPitchTarget(Math.toRadians(95));
                            robot.pinkArm.update();
                        },
                        10000
                )
                .addPoint(
                        "Basket position",
                        new Pose2d(-47.7438, 9.2995, -0.8258),
                        new Pose2d(0.2, 0.2, Math.toRadians(2)),
                        0.3,
                        2000
                )

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
                        "Pick up third sample",
                        new Pose2d(-52.24, 27.586, 0.6838),
                        new Pose2d(0.2, 0.2, Math.toRadians(2)),
                        0.3,
                        2000
                )
                .executeUntilTrue(
                        "Pitch arm up",
                        (Task.Context ctx) -> robot.pinkArm.atTargetPosition(),
                        (Task.Context ctx) -> {
                            robot.pinkArm.setPitchTarget(Math.toRadians(95));
                            robot.pinkArm.update();
                        },
                        10000
                )
                .addPoint(
                        "Basket position",
                        new Pose2d(-47.7438, 9.2995, -0.8258),
                        new Pose2d(0.2, 0.2, Math.toRadians(2)),
                        0.3,
                        2000
                )

                .executeUntilTrue(
                        "Pitch arm up",
                        (Task.Context ctx) -> robot.pinkArm.atTargetPosition(),
                        (Task.Context ctx) -> {
                            robot.pinkArm.setPitchTarget(Math.toRadians(90));
                            robot.pinkArm.update();
                        },
                        10000
                )
                .addPoint(
                        "Ohio Park intermediate",
                        new Pose2d(-31.635, 50.907, 1.4968),
                        new Pose2d(0.2, 0.2, Math.toRadians(2)),
                        0.3,
                        2000
                )
                .executeUntilTrue(
                        "Pitch arm up",
                        (Task.Context ctx) -> robot.pinkArm.atTargetPosition(),
                        (Task.Context ctx) -> {
                            robot.pinkArm.setPitchTarget(Math.toRadians(90));
                            robot.pinkArm.update();
                        },
                        10000
                )
                .addPoint(
                        "Ohio Park",
                        new Pose2d(-13.4556, 53.2135, 1.4968),
                        new Pose2d(0.2, 0.2, Math.toRadians(2)),
                        0.3,
                        2000
                )

                .build();

        follower.follow(this::opModeIsActive, robot.localizer::getPosition);
    }
}
