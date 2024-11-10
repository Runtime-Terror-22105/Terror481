package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.robot.auto.followers.PidToPoint;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

@Autonomous
@Photon
@Config
public class P2PTuning extends LinearOpMode {
    private final RobotHardware hardware = new RobotHardware();
    private final Robot robot = new Robot();
    private FtcDashboard dashboard;

    public static Pose2d goal = new Pose2d(0, 0, 0);
    public static Pose2d tolerances = new Pose2d(0, 0, 0);


    @Override
    public void runOpMode() throws InterruptedException {
        hardware.init(hardwareMap, LynxModule.BulkCachingMode.AUTO);
        robot.init(this, hardware, telemetry);

        PidToPoint p2p = new PidToPoint(goal, tolerances);

        waitForStart();

//        robot.telemetry.addData("time has come", "to tune!");

        while (opModeIsActive()) {
            Pose2d curPos = robot.localizer.getPosition();
            p2p.setGoal(goal, tolerances);
            Pose2d powers = p2p.calculatePower(curPos);
            p2p.driveToDestination(robot.drivetrain, powers, curPos);
            Pose2d curErr = p2p.getError();

            hardware.write();

            robot.telemetry.addData("x pos", curPos.x);
            robot.telemetry.addData("y pos", curPos.y);
            robot.telemetry.addData("h pos", curPos.heading);

            robot.telemetry.addData("x power", powers.x);
            robot.telemetry.addData("y power", powers.y);
            robot.telemetry.addData("h power", powers.heading);

            robot.telemetry.addData("desired x", goal.x);
            robot.telemetry.addData("desired y", goal.y);
            robot.telemetry.addData("desired h", goal.heading);

            robot.telemetry.addData("x err", curErr.x);
            robot.telemetry.addData("y err", curErr.y);
            robot.telemetry.addData("h err", curErr.heading);

            robot.telemetry.addData("x raw pid power", p2p.xTemp);
            robot.telemetry.addData("y raw pid power", p2p.yTemp);

            robot.telemetry.update();
        }
    }
}
