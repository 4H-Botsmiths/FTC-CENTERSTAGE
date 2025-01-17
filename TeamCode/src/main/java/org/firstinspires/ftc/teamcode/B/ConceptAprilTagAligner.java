/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/*
 * This OpMode illustrates the basics of AprilTag recognition and pose estimation,
 * including Java Builder structures for specifying Vision parameters.
 *
 * For an introduction to AprilTags, see the FTC-DOCS link below:
 * https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
 *
 * In this sample, any visible tag ID will be detected and displayed, but only tags that are included in the default
 * "TagLibrary" will have their position and orientation information displayed.  This default TagLibrary contains
 * the current Season's AprilTags and a small set of "test Tags" in the high number range.
 *
 * When an AprilTag in the TagLibrary is detected, the SDK provides location and orientation of the tag, relative to the camera.
 * This information is provided in the "ftcPose" member of the returned "detection", and is explained in the ftc-docs page linked below.
 * https://ftc-docs.firstinspires.org/apriltag-detection-values
 *
 * To experiment with using AprilTags to navigate, try out these two driving samples:
 * RobotAutoDriveToAprilTagOmni and RobotAutoDriveToAprilTagTank
 *
 * There are many "default" VisionPortal and AprilTag configuration parameters that may be overridden if desired.
 * These default parameters are shown as comments in the code below.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp(name = "AprilTag Aligner", group = "B")
public class ConceptAprilTagAligner extends LinearOpMode {

    private static final boolean USE_WEBCAM = true; // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    private Robot robot = null;
    /** How severe the corrections should be */
    final double sensitivity = 0.1;
    int id = 2;
    boolean wasDpadClicked = false;

    @Override
    public void runOpMode() {

        robot = new Robot(hardwareMap);
        initAprilTag();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        while (!opModeIsActive()) {
            telemetryAprilTag();
            telemetry.update();
        }
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if ((gamepad1.dpad_right || gamepad1.dpad_left) && !wasDpadClicked) {
                    wasDpadClicked = true;
                    if (gamepad1.dpad_right) {
                        id++;
                    } else {
                        id--;
                    }
                } else if (!(gamepad1.dpad_right || gamepad1.dpad_left)) {
                    wasDpadClicked = false;
                }
                id = id > 3 ? 3 : id < 1 ? 1 : id;
                telemetry.addData("Target:", id);
                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                AprilTagDetection currentDetection = null;
                for (AprilTagDetection detection : currentDetections) {
                    if (detection.id == id) {
                        currentDetection = detection;
                    }
                }
                if (currentDetection != null) {
                    // ! X, range, yaw

                    Drive(currentDetection.ftcPose.x > 1 ? 0.1 : currentDetection.ftcPose.x < -1 ? -0.1 : 0,
                            currentDetection.ftcPose.range > 20 ? 0.1 : currentDetection.ftcPose.range < 18 ? -0.1 : 0,
                            currentDetection.ftcPose.yaw > 1 ? -0.1
                                    : currentDetection.ftcPose.yaw < -1 ? 0.1 : 0);

                    /*Drive(Range.clip(currentDetection.ftcPose.x * sensitivity, -0.1, 0.1),
                            Range.clip((currentDetection.ftcPose.range - 12) * sensitivity, -0.1, 0.1),
                            Range.clip(currentDetection.ftcPose.yaw * -sensitivity, -0.1, 0.1));*/
                } else if (currentDetections.size() > 0) {
                    for (AprilTagDetection detection : currentDetections) {
                        if (detection.id == 1) {
                            Drive(0.2,
                                    detection.ftcPose.range > 20 ? 0.1
                                            : detection.ftcPose.range < 18 ? -0.1 : 0,
                                    detection.ftcPose.yaw > 1 ? -0.1
                                            : detection.ftcPose.yaw < -1 ? 0.1 : 0);
                            /*Drive(0.15,
                                    Range.clip((currentDetection.ftcPose.range - 12) * sensitivity, -0.1, 0.1),
                                    Range.clip(currentDetection.ftcPose.yaw * -sensitivity, -0.1, 0.1));*/
                        } else if (detection.id == 2) {
                            Drive(id > 2 ? 0.2 : -0.2,
                                    detection.ftcPose.range > 20 ? 0.1
                                            : detection.ftcPose.range < 18 ? -0.1 : 0,
                                    detection.ftcPose.yaw > 1 ? -0.1
                                            : detection.ftcPose.yaw < -1 ? 0.1 : 0);
                        } else if (detection.id == 3) {
                            Drive(-0.2,
                                    detection.ftcPose.range > 20 ? 0.1
                                            : detection.ftcPose.range < 18 ? -0.1 : 0,
                                    detection.ftcPose.yaw > 1 ? -0.1
                                            : detection.ftcPose.yaw < -1 ? 0.1 : 0);
                            /*Drive(-0.15,
                                    Range.clip((currentDetection.ftcPose.range - 12) * sensitivity, -0.1, 0.1),
                                    Range.clip(currentDetection.ftcPose.yaw * -sensitivity, -0.1, 0.1));*/
                        }
                    }
                } else {
                    Drive(0, 0, 0);
                }

                telemetryAprilTag();

                // Push telemetry to the Driver Station.
                telemetry.update();

                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }

                // Share the CPU.
                sleep(20);
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    } // end method runOpMode()

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as
                // needed.
                // .setDrawAxes(false)
                // .setDrawCubeProjection(false)
                // .setDrawTagOutline(true)
                // .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                // .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                // .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                // .setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.
                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 .. Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 .. Detect 2" Tag from 6 feet away at 22 Frames per second
        // Decimation = 3 .. Detect 2" Tag from 4 feet away at 30 Frames Per Second
        // (default)
        // Decimation = 3 .. Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        // aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        // builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView). Set "false" to omit camera monitoring.
        // builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        // builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        // builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);
        builder.enableLiveView(true);
        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        // visionPortal.setProcessorEnabled(aprilTag, true);

    } // end method initAprilTag()

    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x,
                        detection.ftcPose.y, detection.ftcPose.z));

                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch,
                        detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range,
                        detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(
                        String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        } // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    } // end method telemetryAprilTag()

    public void Drive(double x, double y, double z) {
        // r *= steeringMultiplier;
        double m1 = Range.clip(y + x + z, -1, 1);
        double m2 = Range.clip(y - x - z, -1, 1);
        double m3 = Range.clip(y - x + z, -1, 1);
        double m4 = Range.clip(y + x - z, -1, 1);
        // gamepad1.rumble((m1 + m3) / 2, (m2 + m4) / 2,
        // Gamepad.RUMBLE_DURATION_CONTINUOUS);
        /*
         * frontLeft.setVelocity(m1);
         * frontRight.setVelocity(m2);
         * rearLeft.setVelocity(m3);
         * rearRight.setVelocity(m4);
         */
        robot.frontLeft.setSpeed(m1);
        robot.frontRight.setSpeed(m2);
        robot.rearLeft.setSpeed(m3);
        robot.rearRight.setSpeed(m4);

    }
} // end class
