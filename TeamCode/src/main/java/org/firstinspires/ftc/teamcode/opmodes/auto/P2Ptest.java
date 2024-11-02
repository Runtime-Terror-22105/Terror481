//package org.firstinspires.ftc.teamcode.opmodes.auto;
//
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.outoftheboxrobotics.photoncore.Photon;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.math.Pose2d;
//import org.firstinspires.ftc.teamcode.robot.auto.followers.P2PFollower;
//import org.firstinspires.ftc.teamcode.robot.init.Robot;
//import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;
//import org.firstinspires.ftc.teamcode.robot.subsystems.PinkArm;
//
//import java.util.HashMap;
//import java.util.Map;
//
//@Config
//@Photon
//@TeleOp
//public class P2Ptest extends LinearOpMode {
//    private RobotHardware hardware = new RobotHardware();
//    private final Robot robot = new Robot();
//    private FtcDashboard dashboard;
//    @Override
//    public void runOpMode() throws InterruptedException {
//        hardware.init(hardwareMap, LynxModule.BulkCachingMode.AUTO);
//        robot.init(this, hardware, telemetry);
//
//        dashboard = FtcDashboard.getInstance();
//        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
//
//        waitForStart();
//
//        P2PFollower follower = new P2PFollower.Builder(robot.drivetrain)
//                // point, tolerance xyh
//                .addPoint(new Pose2d(1, 43, Math.PI/2), new Pose2d(1, 43, 0.2))
//                // pass in any function
//                .executeActionOnce((HashMap<String, Object> ctx) -> robot.pinkArm.setState(PinkArm.State.HIGH_BASKET))
//                // function returning boolean, function
//                .executeUntilTrue(
//                        (HashMap<String, Object> ctx) -> robot.pinkArm.atTargetPosition(),
//                        (HashMap<String, Object> ctx) -> robot.pinkArm.update()
//                )
//                // NOTE: there is no finishActions() here, so we go to the next point while this action is still ongoing
//                .addPoint(new Pose2d(32, 3, 2), 5)
//                // now, we finally wait for all actions to be finished
//                // basically, all actions are async
//                .finishActions()
//                .build();
//
//        follower.follow();
//    }
//}
