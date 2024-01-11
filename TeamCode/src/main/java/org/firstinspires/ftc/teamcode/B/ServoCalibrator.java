package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Calibrator", group = "B")
public class ServoCalibrator extends LinearOpMode {

  public Servo leftElbow = null;
  public Servo rightElbow = null;

  public Servo trapdoor = null;
  /*
  Pseudo code:
    turn on one motor, check which motor reports moving more than a certain velocity
    repeat with each motor
  */

  @Override
  public void runOpMode() {
    telemetry.addData("Status", "Initializing...");
    telemetry.update();
    leftElbow = hardwareMap.get(Servo.class, "LeftElbow");
    rightElbow = hardwareMap.get(Servo.class, "RightElbow");
    rightElbow.setDirection(Servo.Direction.REVERSE);
    trapdoor = hardwareMap.get(Servo.class, "Trapdoor");
    telemetry.addData("Status", "Initialized - Raise Lift, Then Hit Start");
    waitForStart();
    while (opModeIsActive()) {
      telemetry.addData("Status", "Active");
      updateElbow();
      updateTrapdoor();
      telemetry.update();
    }
  }

  public double elbowPosition = 0.5;

  public void updateElbow() {
    elbowPosition += -(gamepad1.right_stick_y + gamepad2.right_stick_y) * 0.001;
    elbowPosition = elbowPosition > 1 ? 1 : elbowPosition < 0 ? 0 : elbowPosition;
    leftElbow.setPosition(elbowPosition);
    rightElbow.setPosition(elbowPosition);
    telemetry.addData("Elbow", "Left: %d; Right: %d", Math.round(leftElbow.getPosition() * 100),
        Math.round(rightElbow.getPosition() * 100));
  }

  public double trapdoorPosition = 0.5;

  public void updateTrapdoor() {
    trapdoorPosition += -(gamepad1.left_stick_y + gamepad2.left_stick_y) * 0.001;
    trapdoorPosition = trapdoorPosition > 1 ? 1 : trapdoorPosition < 0 ? 0 : trapdoorPosition;
    trapdoor.setPosition(trapdoorPosition);
    telemetry.addData("Trapdoor", Math.round(trapdoor.getPosition() * 100));
  }
}
