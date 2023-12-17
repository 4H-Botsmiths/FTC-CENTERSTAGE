package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@TeleOp(name = "Camera Processor Test", group = "C")
public class CameraProcessorTest extends LinearOpMode {
  // TODO: Add second camera?
  VisionPortal myVisionPortal;
  AprilTagProcessor myAprilTagProcessor;
  TfodProcessor myTfodProcessor;
  boolean USE_WEBCAM;
  String StreamingStatus;
  String LiveViewStatus;

  /**
   * This OpMode allows stopping and resuming only LiveView
   * (RC preview), which does not affect DS Camera Stream.
   * This saves some CPU resources, and resumes very quickly.
   *
   * This OpMode separately allows stopping and resuming Streaming,
   * which includes LiveView and DS Camera Stream. This saves
   * more CPU resources, but takes slightly longer to resume.
   *
   * Another CPU-saving choice: disable/enable the AprilTag and/or TFOD
   * processors.
   *
   * The Telemetry functions include an alternate for
   * getting all or only fresh detections/recognitions.
   *
   * Note the VisionPortal.close() command, to be used when finished using the
   * camera.
   * 
   * Questions, comments and corrections to westsiderobotics@verizon.net
   * 
   */
  @Override
  public void runOpMode() {
    float myFPS;

    // This OpMode turns RC LiveView and Streaming off and on.
    USE_WEBCAM = true;
    StreamingStatus = "ON";
    LiveViewStatus = "ON";
    // Initialize AprilTag before waitForStart.
    initAprilTag();
    // Get the current vision frame rate.
    myFPS = myVisionPortal.getFps();
    while (opModeInInit()) {
      telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
      telemetry.addData(">", "Touch Play to start OpMode");
      telemetry.addLine("");
      Toggle_and_Telemetry();
    }
    // Wait until DS Start button is touched.
    waitForStart();
    if (opModeIsActive()) {
      while (opModeIsActive()) {
        Toggle_and_Telemetry();
      }
      // Save computing resources by closing VisionPortal at any time, if no longer
      // needed.
      myVisionPortal.close();
    }
  }

  /**
   * Describe this function...
   */
  private void Toggle_and_Telemetry() {
    Toggle_LiveView_and_Streaming();
    Toggle_Processors();
    Frames_Per_Second();
    telemetryAprilTag();
    telemetryTfod();
    telemetry.update();
    // Share the CPU.
    sleep(20);
  }

  /**
   * Describe this function...
   */
  private void Frames_Per_Second() {
    // Get the current vision frame rate.
    telemetry.addData("Frames Per Second (FPS)", Double.parseDouble(JavaUtil.formatNumber(myVisionPortal.getFps(), 0)));
    telemetry.addLine("");
  }

  /**
   * Describe this function...
   */
  private void Toggle_LiveView_and_Streaming() {
    if (gamepad1.dpad_left) {
      // Temporarily stop the live view (RC preview).
      myVisionPortal.stopLiveView();
      LiveViewStatus = "OFF";
    } else if (gamepad1.dpad_right) {
      // Start the live view (RC preview) again.
      myVisionPortal.resumeLiveView();
      LiveViewStatus = "ON";
    }
    if (gamepad1.dpad_down) {
      // Temporarily stop the streaming session. This can save CPU
      // resources, with the ability to resume quickly when needed.
      myVisionPortal.stopStreaming();
      StreamingStatus = "OFF";
    } else if (gamepad1.dpad_up) {
      // Resume the streaming session if previously stopped.
      myVisionPortal.resumeStreaming();
      StreamingStatus = "ON";
    }
    telemetry.addLine("Dpad L/R LiveView: " + LiveViewStatus + "    Down/Up Streaming: " + StreamingStatus);
    telemetry.addLine("");
  }

  /**
   * Describe this function...
   */
  private void Toggle_Processors() {
    if (gamepad1.x) {
      // Enable or disable the AprilTag processor.
      myVisionPortal.setProcessorEnabled(myAprilTagProcessor, true);
    } else if (gamepad1.y) {
      // Enable or disable the AprilTag processor.
      myVisionPortal.setProcessorEnabled(myAprilTagProcessor, false);
    }
    if (gamepad1.a) {
      // Enable or disable the TensorFlow Object Detection processor.
      myVisionPortal.setProcessorEnabled(myTfodProcessor, true);
    } else if (gamepad1.b) {
      // Enable or disable the TensorFlow Object Detection processor.
      myVisionPortal.setProcessorEnabled(myTfodProcessor, false);
    }
    // Get whether the AprilTag processor is enabled.
    // Get whether the TensorFlow Object Detection processor is enabled.
    telemetry.addLine("X/Y AprilTag on? " + myVisionPortal.getProcessorEnabled(myAprilTagProcessor)
        + "       A/B TFOD on? " + myVisionPortal.getProcessorEnabled(myTfodProcessor));
    telemetry.addLine("");
  }

  /**
   * Initialize AprilTag Detection.
   */
  private void initAprilTag() {
    // First, create the AprilTag and TFOD Processors.
    myAprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
    // Create the TensorFlow Object Detection processor and assign it to a variable.
    myTfodProcessor = TfodProcessor.easyCreateWithDefaults();
    // Next, create a VisionPortal.
    if (USE_WEBCAM) {
      // Create a VisionPortal, with the specified webcam
      // name, AprilTag processor, and TensorFlow Object
      // Detection processor, and assign it to a variable.
      myVisionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"),
          myAprilTagProcessor, myTfodProcessor);
    } else {
      // Create a VisionPortal, with the specified builtin camera
      // direction, AprilTag processor, and TensorFlow Object
      // Detection processor, and assign it to a variable.
      myVisionPortal = VisionPortal.easyCreateWithDefaults(BuiltinCameraDirection.BACK, myAprilTagProcessor,
          myTfodProcessor);
    }
  }

  /**
   * Display info (using telemetry) for a recognized AprilTag.
   */
  private void telemetryAprilTag() {
    List<AprilTagDetection> myAprilTagDetections;
    AprilTagDetection myAprilTagDetection;

    myAprilTagDetections = myAprilTagProcessor.getDetections();
    // Alternate: get only fresh detections.
    // Get a list containing detections that were detected since the last call to
    // this method, or null if no new detections are available. This is useful to
    // avoid re-processing the same detections multiple times.
    // myAprilTagDetections = myAprilTagProcessor.getFreshDetections();

    if (myAprilTagDetections != null) {

      telemetry.addData("# AprilTags Detected", JavaUtil.listLength(myAprilTagDetections));
      // Iterate through list and call a function to
      // display info for each recognized AprilTag.
      for (AprilTagDetection myAprilTagDetection_item : myAprilTagDetections) {
        myAprilTagDetection = myAprilTagDetection_item;
        // Display info about the detection.
        telemetry.addLine("");
        if (myAprilTagDetection.metadata != null) {
          telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") " + myAprilTagDetection.metadata.name);
          telemetry.addLine("XYZ " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.x, 6, 1) + " "
              + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.y, 6, 1) + " "
              + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.z, 6, 1) + "  (inch)");
          telemetry.addLine("PRY " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.yaw, 6, 1) + " "
              + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.pitch, 6, 1) + " "
              + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.roll, 6, 1) + "  (deg)");
          telemetry.addLine("RBE " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.range, 6, 1) + " "
              + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.bearing, 6, 1) + " "
              + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.elevation, 6, 1) + "  (inch, deg, deg)");
        } else {
          telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") Unknown");
          telemetry.addLine("Center " + JavaUtil.formatNumber(myAprilTagDetection.center.x, 6, 0) + ""
              + JavaUtil.formatNumber(myAprilTagDetection.center.y, 6, 0) + " (pixels)");
        }
      } // end for() loop

    } // end if != null

    telemetry.addLine("");
    telemetry.addLine("key:");
    telemetry.addLine("XYZ = X (Right), Y (Forward), Z (Up) dist.");
    telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
    telemetry.addLine("RBE = Range, Bearing & Elevation");
  }

  /**
   * Display info (using telemetry) for a detected object
   */
  private void telemetryTfod() {
    List<Recognition> myTfodRecognitions;
    Recognition myTfodRecognition;
    float x;
    float y;

    myTfodRecognitions = myTfodProcessor.getRecognitions();
    // Alternate: get only fresh recognitions.
    // Get a list containing recognitions that were detected since the last call to
    // this method, or null if no new recognitions are available. This is useful to
    // avoid re-processing the same recognitions multiple times.
    // myTfodRecognitions = myTfodProcessor.getFreshRecognitions();

    if (myTfodRecognitions != null) {

      telemetry.addData("# Objects Detected", JavaUtil.listLength(myTfodRecognitions));
      // Iterate through list and call a function to
      // display info for each recognized object.
      for (Recognition myTfodRecognition_item : myTfodRecognitions) {
        myTfodRecognition = myTfodRecognition_item;
        // Display info about the recognition.
        telemetry.addLine("");
        // Display label and confidence.
        // Display the label and confidence for the recognition.
        telemetry.addData("Image", myTfodRecognition.getLabel() + " ("
            + JavaUtil.formatNumber(myTfodRecognition.getConfidence() * 100, 0) + " % Conf.)");
        // Display position.
        x = (myTfodRecognition.getLeft() + myTfodRecognition.getRight()) / 2;
        y = (myTfodRecognition.getTop() + myTfodRecognition.getBottom()) / 2;
        // Display the position of the center of the detection boundary for the
        // recognition
        telemetry.addData("- Position", JavaUtil.formatNumber(x, 0) + ", " + JavaUtil.formatNumber(y, 0));
        // Display size
        // Display the size of detection boundary for the recognition
        telemetry.addData("- Size", JavaUtil.formatNumber(myTfodRecognition.getWidth(), 0) + " x "
            + JavaUtil.formatNumber(myTfodRecognition.getHeight(), 0));
      } // end for() loop

    } // end if != null

  }
}