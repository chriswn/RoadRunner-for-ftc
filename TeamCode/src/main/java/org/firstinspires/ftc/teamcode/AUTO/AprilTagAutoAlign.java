package org.firstinspires.ftc.teamcode.AUTO;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "AprilTag Auto-Align")
public class AprilTagAutoAlign extends LinearOpMode {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // Define motors
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // PID tuning values (adjust as needed)
    private final double STRAFE_Kp = 0.02;
    private final double FORWARD_Kp = 0.02;
    private final double TURN_Kp = 0.01;

    @Override
    public void runOpMode() {
        initHardware();
        initAprilTag();

        telemetry.addData("Status", "Initialized. Waiting for start...");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                AprilTagDetection bestTag = getClosestTag();
                if (bestTag != null) {
                    alignToTag(bestTag);
                } else {
                    telemetry.addData("AprilTag", "No tag detected.");
                }
                telemetry.update();
                sleep(50);
            }
        }
        visionPortal.close();
    }

    private void initHardware() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    private AprilTagDetection getClosestTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections.isEmpty()) return null;

        AprilTagDetection closestTag = detections.get(0);
        for (AprilTagDetection tag : detections) {
            if (Math.abs(tag.ftcPose.y) < Math.abs(closestTag.ftcPose.y)) {
                closestTag = tag;
            }
        }
        return closestTag;
    }

    private void alignToTag(AprilTagDetection tag) {
        double xError = tag.ftcPose.x;  // Left/Right
        double yError = tag.ftcPose.y;  // Forward/Backward
        double yawError = tag.ftcPose.yaw;  // Rotation

        double strafePower = xError * STRAFE_Kp;
        double forwardPower = yError * FORWARD_Kp;
        double turnPower = yawError * TURN_Kp;

        // Clip values to avoid excessive power
        strafePower = Math.max(-0.5, Math.min(0.5, strafePower));
        forwardPower = Math.max(-0.5, Math.min(0.5, forwardPower));
        turnPower = Math.max(-0.3, Math.min(0.3, turnPower));

        moveRobot(forwardPower, strafePower, turnPower);

        telemetry.addData("Aligning to Tag", "ID: %d", tag.id);
        telemetry.addData("X Error", "%.2f", xError);
        telemetry.addData("Y Error", "%.2f", yError);
        telemetry.addData("Yaw Error", "%.2f", yawError);
    }

    private void moveRobot(double forward, double strafe, double turn) {
        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;

        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }
}
