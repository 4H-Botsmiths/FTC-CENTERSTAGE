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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.hardware.Camera;
import org.firstinspires.ftc.teamcode.hardware.Camera.AprilTagPosition;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import static org.firstinspires.ftc.teamcode.classes.AutoGlobals.*;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Backstage Red", group = "B", preselectTeleOp = "Camera Teleop")
public class BackstageRedAuto extends LinearOpMode {
  public Robot robot = null;
  public Camera camera = null;

  /*
  Pseudo code:
    delay (time based off of pre-game user input)
    strafe right
    align & place on board (position based off of pre-game user input)
    backup
    park inside stripe next to board (position based off of pre-game user input)
      if left, 
        constrict();
        drive into side wall then drive forward into wall
      if right, 
        align to side of board using camera (will have to back up to keep apriltag in frame)
        constrict();
        drive forward into wall
  */

  public boolean delay = false;
  public AprilTagPosition placingPosition = AprilTagPosition.CENTER;
  public ParkingLocation parkingLocation = ParkingLocation.RIGHT;

  enum ParkingLocation {
    LEFT, RIGHT
  }

  @Override
  public void runOpMode() {
    telemetry.addData("Status", "Initializing...");
    telemetry.update();
    robot = new Robot(hardwareMap);
    camera = new Camera(hardwareMap, telemetry);
    try {
      camera.initAprilTag();
    } catch (Camera.CameraNotAttachedException e) {
      telemetry.speak("Warning: Camera is not attached");
    }
    while (opModeInInit()) {
      initInput();
      initTelemetries();
      sleep(100);
    }
    //OpMode is now started
    if (delay) {
      telemetry.speak("Delaying");
      sleep(DELAY_DURATION);
    }
    telemetry.speak("Strafing");
    robot.Drive(-PRIMARY_SPEED, 0, 0);
    sleep(STRAFE_DURATION);
    robot.Drive(0, 0, 0);
    robot.lift.expand();
    robot.Drive(0, TERTIARY_SPEED, 0);
    sleep(CREEP_DURATION);
    align(placingPosition);
    approach(placingPosition);
    place(placingPosition);
    robot.Drive(0, -PRIMARY_SPEED, 0);
    telemetry.speak("Backing up");
    sleep(BACKUP_DURATION);
    robot.intake.hold();
    if (parkingLocation == ParkingLocation.LEFT) {
      telemetry.speak("Parking left");
      robot.Drive(0, -SECONDARY_SPEED, 0);
      sleep(LETS_BACK_UP);
    } else {
      robot.Drive(SECONDARY_SPEED, 0, 0);
      telemetry.speak("Parking right");
      sleep(PARKING_STRAFE_DURATION);
      robot.lift.constrict();
      telemetry.speak("Compressing Lift");
      robot.Drive(0, 0, 0);
      while (robot.lift.status != Robot.LiftStatus.LOWERED) {
        sleep(50);
      }
      robot.lift.setSpeed(POPUP_SPEED);
      sleep(POPUP_DURATION);
      robot.lift.setSpeed(0);
      robot.Drive(0, SECONDARY_SPEED, 0);
      telemetry.speak("Approaching Wall");
      sleep(PARKING_DURATION);
    }

    robot.Drive(0, 0, 0);
    telemetry.speak("Done");
  }

  public void initTelemetries() {
    telemetry.addData("Status", "Waiting for start");
    telemetry.addData("Delaying (A/B)", delay);
    telemetry.addData("Placement Position (Dpad Left/Up/Right)", placingPosition);
    telemetry.addData("Parking Location (Left/Right Bumpers)", parkingLocation);
    telemetry.update();
  }

  public void initInput() {
    if (gamepad1.left_bumper || gamepad2.left_bumper) {
      parkingLocation = ParkingLocation.LEFT;
    } else if (gamepad1.right_bumper || gamepad2.right_bumper) {
      parkingLocation = ParkingLocation.RIGHT;
    }
    if (gamepad1.dpad_up || gamepad2.dpad_up) {
      placingPosition = AprilTagPosition.CENTER;
    } else if (gamepad1.dpad_left || gamepad2.dpad_left) {
      placingPosition = AprilTagPosition.LEFT;
    } else if (gamepad1.dpad_right || gamepad2.dpad_right) {
      placingPosition = AprilTagPosition.RIGHT;
    }
    if (gamepad1.a || gamepad2.a) {
      delay = true;
    } else if (gamepad1.b || gamepad2.b) {
      delay = false;
    }
  }

  public void align(AprilTagPosition position) {
    telemetry.speak("Aligning");
    while (opModeIsActive()) {
      try {
        List<Camera.AprilTag> tags = camera.getAprilTags();

        Camera.AprilTag tag = null;
        for (Camera.AprilTag _tag : tags) {
          if (_tag.position == position) {
            tag = _tag;
          }
        }
        if (tag != null) {
          if (tag.ftcPose.x < tolerance && tag.ftcPose.x > -tolerance
              && tag.ftcPose.range < DISTANCE + tolerance
              && tag.ftcPose.yaw < tolerance && tag.ftcPose.yaw > -tolerance) {
            return;
          }
          telemetry.addLine("Aligning");
          robot.Drive(Range.clip(tag.ftcPose.x * sensitivity, -speedLimit, speedLimit),
              Range.clip((tag.ftcPose.range - DISTANCE) * sensitivity, -speedLimit, speedLimit),
              Range.clip(tag.ftcPose.yaw * -turnSensitivity, -turnSpeedLimit, turnSpeedLimit));
        } else {
          for (Camera.AprilTag _tag : tags) {
            if (_tag.position != Camera.AprilTagPosition.CENTER) {
              tag = _tag;
            }
          }
          tag = tag == null ? tags.get(0) : tag;
          switch (position) {
            case LEFT:
              robot.Drive(
                  (tag.position == Camera.AprilTagPosition.CENTER ? -0.2 : -0.3),
                  Range.clip((tag.ftcPose.range - DISTANCE) * sensitivity, -speedLimit, speedLimit),
                  Range.clip(tag.ftcPose.yaw * -turnSensitivity, -turnSpeedLimit, turnSpeedLimit));
              break;
            case RIGHT:
              robot.Drive((tag.position == Camera.AprilTagPosition.CENTER ? 0.2 : 0.3),
                  Range.clip((tag.ftcPose.range - DISTANCE) * sensitivity, -speedLimit, speedLimit),
                  Range.clip(tag.ftcPose.yaw * -turnSensitivity, -turnSpeedLimit, turnSpeedLimit));
              break;
            case CENTER:
              robot.Drive((tag.position == Camera.AprilTagPosition.LEFT ? 0.2 : -0.2),
                  Range.clip((tag.ftcPose.range - DISTANCE) * sensitivity, -speedLimit, speedLimit),
                  Range.clip(tag.ftcPose.yaw * -turnSensitivity, -turnSpeedLimit, turnSpeedLimit));
              break;
            default:
              robot.Drive(0, 0, 0);
          }
        }
      } catch (Camera.CameraNotStreamingException e) {
        //Do nothing, the camera should be starting
      } catch (Camera.NoTagsFoundException e) {
        robot.Drive(0, 0, 0);
      } catch (Camera.CameraNotAttachedException e) {
        //Function can't run
        telemetry.speak("Camera is not attached");
        return;
      }
    }
  }

  public void approach(AprilTagPosition position) {
    telemetry.speak("Approaching");
    while (opModeIsActive()) {
      robot.intake.hold();
      try {
        List<Camera.AprilTag> tags = camera.getAprilTags();

        Camera.AprilTag tag = null;
        for (Camera.AprilTag _tag : tags) {
          if (_tag.position == position) {
            tag = _tag;
          }
        }
        if (tag != null) {
          if (tag.ftcPose.x < precisionTolerance && tag.ftcPose.x > -precisionTolerance
              && tag.ftcPose.range < FINAL_DISTANCE + precisionTolerance
              && tag.ftcPose.yaw < precisionTolerance && tag.ftcPose.yaw > -precisionTolerance) {
            return;
          }
          telemetry.addLine("Approaching");
          robot.Drive(Range.clip(tag.ftcPose.x * sensitivity, -precisionSpeedLimit, precisionSpeedLimit),
              Range.clip(((tag.ftcPose.range - FINAL_DISTANCE) * sensitivity) + pressureSpeed, -precisionSpeedLimit,
                  precisionSpeedLimit),
              Range.clip(tag.ftcPose.yaw * -turnSensitivity, -turnSpeedLimit, turnSpeedLimit));
        } else {
          align(position);
        }
      } catch (Camera.CameraNotStreamingException e) {
        //Do nothing, the camera should be starting
      } catch (Camera.NoTagsFoundException e) {
        robot.Drive(0, 0, 0);
      } catch (Camera.CameraNotAttachedException e) {
        //Function can't run
        telemetry.speak("Camera is not attached");
        return;
      }
    }
  }

  ElapsedTime timer = new ElapsedTime();

  public void place(AprilTagPosition position) {
    timer.reset();
    telemetry.speak("Placing");
    while (opModeIsActive() && timer.milliseconds() < PLACE_DURATION) {
      robot.intake.drop();
      try {
        List<Camera.AprilTag> tags = camera.getAprilTags();

        Camera.AprilTag tag = null;
        for (Camera.AprilTag _tag : tags) {
          if (_tag.position == position) {
            tag = _tag;
          }
        }
        if (tag != null) {
          /*if (tag.ftcPose.x < precisionTolerance && tag.ftcPose.x > -precisionTolerance
              && tag.ftcPose.range < FINAL_DISTANCE + precisionTolerance
              && tag.ftcPose.yaw < precisionTolerance && tag.ftcPose.yaw > -precisionTolerance) {
            return;
          }*/
          telemetry.addLine("Placing");
          robot.Drive(Range.clip(tag.ftcPose.x * sensitivity, -precisionSpeedLimit, precisionSpeedLimit),
              Range.clip(((tag.ftcPose.range - FINAL_DISTANCE) * sensitivity) + pressureSpeed, -precisionSpeedLimit,
                  precisionSpeedLimit),
              Range.clip(tag.ftcPose.yaw * -turnSensitivity, -turnSpeedLimit, turnSpeedLimit));
        } else {
          approach(position);
        }
      } catch (Camera.CameraNotStreamingException e) {
        //Do nothing, the camera should be starting
      } catch (Camera.NoTagsFoundException e) {
        robot.Drive(0, 0, 0);
      } catch (Camera.CameraNotAttachedException e) {
        //Function can't run
        telemetry.speak("Camera is not attached");
        return;
      }
    }
  }

  public void prepareParkLeft() {
    while (opModeIsActive()) {
      try {
        List<Camera.AprilTag> tags = camera.getAprilTags();

        Camera.AprilTag tag = null;
        for (Camera.AprilTag _tag : tags) {
          if (_tag.position == Camera.AprilTagPosition.LEFT) {
            tag = _tag;
          }
        }
        if (tag != null) {
          if (tag.ftcPose.x + PARKING_X_DISTANCE < tolerance && tag.ftcPose.x + PARKING_X_DISTANCE > -tolerance
              && tag.ftcPose.range < DISTANCE + tolerance
              && tag.ftcPose.yaw < tolerance && tag.ftcPose.yaw > -tolerance) {
            return;
          }
          robot.Drive(Range.clip(tag.ftcPose.x + PARKING_X_DISTANCE * sensitivity, -speedLimit, speedLimit),
              Range.clip((tag.ftcPose.range - DISTANCE) * sensitivity, -speedLimit, speedLimit),
              Range.clip(tag.ftcPose.yaw * -turnSensitivity, -turnSpeedLimit, turnSpeedLimit));
        } else {
          for (Camera.AprilTag _tag : tags) {
            if (_tag.position != Camera.AprilTagPosition.CENTER) {
              tag = _tag;
            }
          }
          tag = tag == null ? tags.get(0) : tag;
          robot.Drive((tag.position == Camera.AprilTagPosition.CENTER ? -0.2 : -0.3),
              Range.clip((tag.ftcPose.range - DISTANCE) * sensitivity, -speedLimit, speedLimit),
              Range.clip(tag.ftcPose.yaw * -turnSensitivity, -turnSpeedLimit, turnSpeedLimit));
        }
      } catch (Camera.CameraNotStreamingException e) {
        //Do nothing, the camera should be starting
      } catch (Camera.NoTagsFoundException e) {
        robot.Drive(0, 0, 0);
      } catch (Camera.CameraNotAttachedException e) {
        //Function can't run
        telemetry.speak("Camera is not attached");
        return;
      }
    }
  }
}
