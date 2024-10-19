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
import org.firstinspires.ftc.teamcode.robot.hardware.TerrorPublisher;
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

    // Pink arm stuff
//    public TerrorMotor armPitchMotor1 = null;
//    public TerrorMotor armPitchMotor2 = null;
//    public TerrorAnalogEncoder armPitchEncoder = null;
//    public TerrorMotor armExtensionMotor1 = null;
//    public TerrorMotor armExtensionMotor2 = null;
//    public TerrorEncoder armExtensionEncoder = null;

    // In/Out take stuff
//    public TerrorServo wheelPitchServo1 = null;
//    public TerrorServo wheelPitchServo2 = null;
//    public TerrorCRServo wheelRotationServoLeft = null;
//    public TerrorCRServo wheelRotationServoRight = null;
//    public TerrorColorRangeFinder wheelColorSensor = null;

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
    private TerrorPublisher publisher = new TerrorPublisher();

    public void init(@NonNull HardwareMap hwMap, @NonNull LynxModule.BulkCachingMode bulkCachingMode) {
        this.hwMap = hwMap;

        // Initialize the drivetrain motors
        this.motorFrontLeft = new TerrorMotor(
                ((PhotonDcMotor) hwMap.get(DcMotor.class, "motorFrontLeft")),
                0.05
        );
        this.motorFrontRight = new TerrorMotor(
                ((PhotonDcMotor) hwMap.get(DcMotor.class, "motorFrontRight")),
                0.05
        );
        this.motorRearRight = new TerrorMotor(
                ((PhotonDcMotor) hwMap.get(DcMotor.class, "motorRearRight")),
                0.05
        );
        this.motorRearLeft = new TerrorMotor(
                ((PhotonDcMotor) hwMap.get(DcMotor.class, "motorRearLeft")),
                0.05
        );
        this.motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorRearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorRearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.publisher.subscribe(4, motorFrontLeft, motorFrontRight, motorRearLeft, motorRearRight);


        // Initialize the pink arm motors and sensors
//        this.armPitchMotor1 = new TerrorMotor(
//                ((PhotonDcMotor)hwMap.get(DcMotor.class, "armPitchMotor1")),
//                0.02
//        );
//        this.armPitchMotor2 = new TerrorMotor(
//                ((PhotonDcMotor)hwMap.get(DcMotor.class, "armPitchMotor2")),
//                0.02
//        );
//        this.armPitchEncoder = new TerrorAnalogEncoder(hwMap.get(AnalogInput.class, "armPitchEncoder"));
//        this.armPitchEncoder.setOffset(0);
//        this.armExtensionMotor1 = new TerrorMotor(
//                ((PhotonDcMotor) hwMap.get(DcMotor.class, "armExtensionMotor1")),
//                0.02
//        );
//        this.armExtensionMotor2 = new TerrorMotor(
//                ((PhotonDcMotor) hwMap.get(DcMotor.class, "armExtensionMotor2")),
//                0.02
//        );
//        this.armExtensionEncoder = new TerrorEncoder(armExtensionMotor1); // might need to change to motor 2
//        this.publisher.subscribe(5, armPitchMotor1, armPitchMotor2);
//        this.publisher.subscribe(3, armExtensionMotor1, armExtensionMotor2);
//
//        // Initialize the inouttake servos and sensors
//        this.wheelPitchServo1 = new TerrorServo((PhotonServo) hwMap.get(Servo.class, "wheelPitchServo1"));
//        this.wheelPitchServo2 = new TerrorServo((PhotonServo) hwMap.get(Servo.class, "wheelPitchServo2"));
//        this.wheelRotationServoLeft = new TerrorCRServo((PhotonCRServo)
//                hwMap.get(CRServo.class, "wheelRotationServoLeft"), 0.02);
//        this.wheelRotationServoRight = new TerrorCRServo((PhotonCRServo)
//                hwMap.get(CRServo.class, "wheelRotationServoRight"), 0.02);
//        this.wheelColorSensor = new TerrorColorRangeFinder(
//            hwMap.digitalChannel.get("digital0"),
//            hwMap.digitalChannel.get("digital1")
//        ); // assume that the color sensor is already configured
//        this.publisher.subscribe(1, wheelPitchServo1, wheelPitchServo2);
//        this.publisher.subscribe(2, wheelRotationServoLeft, wheelRotationServoRight);

//        this.initCamera();
        this.initLynx(bulkCachingMode);

        // Misc Sensors
//        this.otos = this.hwMap.get(TerrorSparkFunOTOS.class, "sensor_otos");
        this.voltageSensor = hwMap.getAll(PhotonLynxVoltageSensor.class).iterator().next();
    }

    public void write() {
        this.publisher.write();
    }

    private void initLynx(LynxModule.BulkCachingMode bulkCachingMode) {
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

        if (this.camera.tagProcessor == null) {
            throw new IllegalStateException("AprilTag processor not initialized!");
        }
//        this.camera.tagProcessor.setDecimation(???); // TODO: tune decimation value
        this.camera.tagProcessor.setPoseSolver(AprilTagProcessor.PoseSolver.APRILTAG_BUILTIN);
    }
}
