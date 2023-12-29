package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.concurrent.CompletableFuture;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionPortal.CameraState;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.stream.Collectors;

/*
 * This OpMode illustrates the basics of AprilTag recognition and pose estimation, using
 * two webcams.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
public class Camera {
  private Telemetry telemetry;

  /*
   * Variables used for switching cameras.
   */
  public Camera(HardwareMap hardwareMap, Telemetry telemetry) {
    this.webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
    this.telemetry = telemetry;
  }

  private WebcamName webcam;
  /**
   * The variable to store our instance of the AprilTag processor.
   */
  private AprilTagProcessor aprilTag;

  /**
   * The variable to store our instance of the vision portal.
   */
  public VisionPortal visionPortal;

  /**
   * Initialize the AprilTag processor.
   */
  public void initAprilTag() throws CameraNotAttachedException {
    if (!webcam.isAttached()) {
      throw new CameraNotAttachedException();
    }

    // Create the AprilTag processor by using a builder.
    aprilTag = new AprilTagProcessor.Builder().build();
    // Create the vision portal by using a builder.
    visionPortal = new VisionPortal.Builder()
        .setCamera(this.webcam)
        .addProcessor(aprilTag)
        .build();

  } // end method initAprilTag()

  /**
   * Retrieves the list of AprilTags detected by the camera.
   *
   * @return         	A list of AprilTag objects representing the detected tags.
   * @throws CameraNotStreamingException  If the camera is not currently streaming.
   * @throws NoTagsFoundException         If no AprilTags are found in the current frame.
   * @throws CameraNotAttachedException   If the camera is not attached.
   */
  public List<AprilTag> getAprilTags()
      throws CameraNotStreamingException, NoTagsFoundException, CameraNotAttachedException {
    if (!webcam.isAttached()) {
      throw new CameraNotAttachedException();
    } else if (visionPortal == null) {
      initAprilTag();
    }
    if (visionPortal.getCameraState() != CameraState.STREAMING) {
      throw new CameraNotStreamingException();
    }
    List<AprilTagDetection> currentDetections = aprilTag.getDetections();
    if (currentDetections.size() == 0) {
      throw new NoTagsFoundException();
    }
    return currentDetections.stream().map(aprilTag -> new AprilTag(aprilTag)).collect(Collectors.toList());
  }

  public void pause() throws CameraNotAttachedException {
    if (!webcam.isAttached()) {
      throw new CameraNotAttachedException();
    } else if (visionPortal == null) {
      initAprilTag();
    }
    if (this.visionPortal.getCameraState() == CameraState.OPENING_CAMERA_DEVICE) {
      //You can't stop a camera stream before the device is opened
      CompletableFuture.runAsync(() -> {
        while (this.visionPortal.getCameraState() == CameraState.OPENING_CAMERA_DEVICE) {
          try {
            Thread.sleep(100);
          } catch (InterruptedException e) {
            //Do nothing
          }
        }
        visionPortal.stopStreaming();
      });
    } else {
      try {
        visionPortal.stopStreaming();
      } catch (Exception e) {
        //This function has called errors before
        telemetry.speak("Error pausing camera");
      }
    }
  }

  public void resume() throws CameraNotAttachedException {
    if (!webcam.isAttached()) {
      throw new CameraNotAttachedException();
    } else if (visionPortal == null) {
      initAprilTag();
    }
    visionPortal.resumeStreaming();
  }

  /**
   * Add telemetry about AprilTag detections.
   */
  public void telemetryAprilTag() {
    try {
      List<AprilTag> currentDetections = getAprilTags();
      telemetry.addData("# AprilTags Detected", currentDetections.size());

      // Step through the list of detections and display info for each one.
      for (AprilTag detection : currentDetections) {
        if (detection.metadata != null) {
          telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
          telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y,
              detection.ftcPose.z));
          telemetry
              .addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll,
                  detection.ftcPose.yaw));
          telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range,
              detection.ftcPose.bearing, detection.ftcPose.elevation));
        } else {
          telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
          telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
        }
      } // end for() loop

      // Add "key" information to telemetry
      telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
      telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
      telemetry.addLine("RBE = Range, Bearing & Elevation");
    } catch (CameraNotAttachedException e) {
      telemetry.addLine("Camera is not attached");
    } catch (CameraNotStreamingException e) {
      telemetry.addLine("Camera is not streaming");
      telemetry.addLine(visionPortal.getCameraState().toString());
    } catch (NoTagsFoundException e) {
      telemetry.addLine("No AprilTags Detected");
    }
  } // end method telemetryAprilTag()

  /*
   * public enum ActiveCamera {
   * LEFT, RIGHT, NONE
   * }
   * 
   * public void setCamera(ActiveCamera camera) {
   * if (camera == ActiveCamera.LEFT) {
   * visionPortal.resumeStreaming();
   * visionPortal.setActiveCamera(webcam1);
   * } else if (camera == ActiveCamera.RIGHT) {
   * visionPortal.resumeStreaming();
   * visionPortal.setActiveCamera(webcam2);
   * } else if (camera == ActiveCamera.NONE) {
   * visionPortal.stopStreaming();
   * }
   * }
   */
  public class NoTagsFoundException extends Exception {
    public NoTagsFoundException() {
      super("No tags were found");
    }
  }

  public class CameraNotStreamingException extends Exception {
    public CameraNotStreamingException() {
      super("The camera is not streaming");
    }
  }

  public class CameraNotAttachedException extends Exception {
    public CameraNotAttachedException() {
      super("The camera is not attached");
    }
  }

  public enum AprilTagPosition {
    LEFT, CENTER, RIGHT, UNKNOWN
  }

  public class AprilTag extends AprilTagDetection {
    public AprilTagPosition position = AprilTagPosition.UNKNOWN;

    public AprilTag(AprilTagDetection detection) {
      //AprilTagDetection​(int id, int hamming, float decisionMargin, org.opencv.core.Point center, org.opencv.core.Point[] corners, AprilTagMetadata metadata, AprilTagPoseFtc ftcPose, AprilTagPoseRaw rawPose, long frameAcquisitionNanoTime)
      super(detection.id, detection.hamming, detection.decisionMargin, detection.center, detection.corners,
          detection.metadata,
          detection.metadata != null
              ? new AprilTagPoseFtc(detection.ftcPose.x - 0.6, detection.ftcPose.y, detection.ftcPose.z,
                  detection.ftcPose.yaw + 1.87,
                  detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.range - 9.8,
                  detection.ftcPose.bearing,
                  detection.ftcPose.elevation)
              : new AprilTagPoseFtc(0, 0, 0, 0, 0, 0, 0, 0, 0),
          detection.rawPose, detection.frameAcquisitionNanoTime);
      if (detection.metadata == null) {
        throw new IllegalStateException(String.format("Detection with ID: %d has no metadata", detection.id));
      }
      if (detection.id == 1 || detection.id == 4) {
        position = AprilTagPosition.LEFT;
      }
      if (detection.id == 2 || detection.id == 5) {
        position = AprilTagPosition.CENTER;
      }
      if (detection.id == 3 || detection.id == 6) {
        position = AprilTagPosition.RIGHT;
      }
    }
  }
} // end class
