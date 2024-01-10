package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "Drivetrain Diagnostics", group = "B")
public class DrivetrainDiagnostics extends LinearOpMode {

  public DcMotorEx frontLeft = null;
  public DcMotorEx frontRight = null;
  public DcMotorEx rearLeft = null;
  public DcMotorEx rearRight = null;

  public final int RUN_DURATION = 1000;
  public final int MINIMUM_VELOCITY = 100;
  /*
  Pseudo code:
    turn on one motor, check which motor reports moving more than a certain velocity
    repeat with each motor
  */

  @Override
  public void runOpMode() {
    telemetry.setAutoClear(false);
    telemetry.addData("Status", "Initializing...");
    telemetry.addData("Front Left", "Waiting...");
    telemetry.addData("Front Right", "Waiting...");
    telemetry.addData("Rear Left", "Waiting...");
    telemetry.addData("Rear Right", "Waiting...");
    telemetry.update();
    frontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeft");
    frontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
    rearLeft = hardwareMap.get(DcMotorEx.class, "RearLeft");
    rearRight = hardwareMap.get(DcMotorEx.class, "RearRight");
    telemetry.addData("Status", "Initialized");
    telemetry.update();
    waitForStart();
    telemetry.addData("Status", "Testing Motors...");
    evaluateMotor(frontLeft, "Front Left");
    evaluateMotor(frontRight, "Front Right");
    evaluateMotor(rearLeft, "Rear Left");
    evaluateMotor(rearRight, "Rear Right");
    resetMotors();
    telemetry.addData("Status", "Done, See Below For Test Results");
    telemetry.update();
  }

  public void resetMotors() {
    //Make sure that the encoders won't be used when calling setPower()
    frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    rearLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    rearRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    //Set the motors to brake so that the last motor moved wont trigger a false active
    frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    rearLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    rearRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    //Stop all the motors
    frontLeft.setPower(0);
    frontRight.setPower(0);
    rearLeft.setPower(0);
    rearRight.setPower(0);
  }

  public DcMotorEx getActiveMotor() {
    if (frontLeft.getVelocity() > MINIMUM_VELOCITY) {
      return frontLeft;
    } else if (frontRight.getVelocity() > MINIMUM_VELOCITY) {
      return frontRight;
    } else if (rearLeft.getVelocity() > MINIMUM_VELOCITY) {
      return rearLeft;
    } else if (rearRight.getVelocity() > MINIMUM_VELOCITY) {
      return rearRight;
    } else {
      return null;
    }
  }

  public void evaluateMotor(DcMotorEx motor, String motorName) {
    telemetry.addData(motorName, "Testing...");
    telemetry.update();
    resetMotors();
    motor.setPower(1);
    sleep(RUN_DURATION);
    DcMotorEx activeMotor = getActiveMotor();
    if (activeMotor == null) {
      telemetry.addData(motorName, "Encoder Error");
    } else if (activeMotor != motor) {
      telemetry.addData(motorName, "Encoder Mismatch, Encoder Connected To Port %i, should be %i",
          activeMotor.getPortNumber(), motor.getPortNumber());
    } else {
      telemetry.addData(motorName, "Encoder Functioning (Velocity: %.0f)", motor.getVelocity());
    }
    telemetry.update();
  }
}
