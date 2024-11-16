package org.firstinspires.ftc.teamcode.opmodes.auto;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.auto.followers.PurePursuitController;
import org.firstinspires.ftc.teamcode.robot.auto.pathgen.QSplines;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

@Config
@TeleOp
public class SampleAutonomous extends LinearOpMode {
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

        QSplines spline = new QSplines(0,0,0, 0,5, 0, 0, 0);

        PurePursuitController controller = new PurePursuitController(robot.drivetrain, 2);
//        controller.normalFollow(() -> robot.localizer.getPosition(), 5);
        controller.follow(() -> robot.localizer.getPosition());
    }
}
