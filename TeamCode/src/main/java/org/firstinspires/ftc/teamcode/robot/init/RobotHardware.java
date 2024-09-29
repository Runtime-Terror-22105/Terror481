package org.firstinspires.ftc.teamcode.robot.init;

import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.outoftheboxrobotics.photoncore.Photon;
import com.outoftheboxrobotics.photoncore.hardware.PhotonLynxVoltageSensor;
import com.outoftheboxrobotics.photoncore.hardware.motor.PhotonDcMotor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robot.hardware.motors.TerrorMotor;
import org.firstinspires.ftc.teamcode.robot.hardware.sensors.TerrorSparkFunOTOS;
import org.firstinspires.ftc.teamcode.robot.hardware.sensors.camera.TerrorCamera;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * A class containing all the robot's hardware.
 */
@Config
@Photon
public class RobotHardware {

    // Drivetrain motors & servos
    public TerrorMotor motorFrontLeft = null;
    public TerrorMotor motorRearRight = null;
    public TerrorMotor motorFrontRight = null;
    public TerrorMotor motorRearLeft = null;

    // Camera
    private TerrorCamera camera;

    // Sensors
    public PhotonLynxVoltageSensor voltageSensor;
    public TerrorSparkFunOTOS otos;

    // Lynx stuff
    public List<LynxModule> allHubs = null;
    public LynxModule controlHub = null;

    // Other
    public HardwareMap hwMap;

    public void init(@NonNull HardwareMap hwMap, @NonNull LynxModule.BulkCachingMode bulkCachingMode) {
        this.hwMap = hwMap;

        // Initialize the drivetrain motors
        this.motorFrontLeft  = new TerrorMotor(((PhotonDcMotor)hwMap.get(DcMotor.class, "motorFrontLeft")));
        this.motorFrontRight = new TerrorMotor(((PhotonDcMotor)hwMap.get(DcMotor.class, "motorFrontRight")));
        this.motorRearRight  = new TerrorMotor(((PhotonDcMotor)hwMap.get(DcMotor.class, "motorRearRight")));
        this.motorRearLeft   = new TerrorMotor(((PhotonDcMotor)hwMap.get(DcMotor.class, "motorRearLeft")));
        this.motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorRearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorRearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.initCamera();
        this.initLynx(bulkCachingMode);

        // Sensor
        this.otos = this.hwMap.get(TerrorSparkFunOTOS.class, "sensor_otos");
        this.voltageSensor = hwMap.getAll(PhotonLynxVoltageSensor.class).iterator().next();
    }

    public void initLynx(LynxModule.BulkCachingMode bulkCachingMode) {
        // Initialize Lynx stuff
        this.allHubs = this.hwMap.getAll(LynxModule.class);
        for (LynxModule hub : this.allHubs) {
            if (hub.isParent() && LynxConstants.isEmbeddedSerialNumber(hub.getSerialNumber())) {
                this.controlHub = hub;
            }
            hub.setBulkCachingMode(bulkCachingMode);
        }

    }

    private void initCamera() {
        int cameraMonitorViewId = hwMap
                .appContext
                .getResources()
                .getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hwMap.appContext.getPackageName()
                );
        WebcamName cameraName = hwMap.get(WebcamName.class, "Webcam 1");

        this.camera = new TerrorCamera.VisionPortalInitialization()
                .setCamera(cameraName)
                .setCameraResolution(new Size(1280, 800))
                .detectAprilTags()
                .init();

        assert this.camera.tagProcessor != null;
//        this.camera.tagProcessor.setDecimation(???); // TODO: tune decimation value
        this.camera.tagProcessor.setPoseSolver(AprilTagProcessor.PoseSolver.APRILTAG_BUILTIN);
    }
}
