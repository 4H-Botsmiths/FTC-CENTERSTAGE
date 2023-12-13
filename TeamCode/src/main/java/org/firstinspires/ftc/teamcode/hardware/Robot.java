package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.classes.DCMotor;

public class Robot {
  public DCMotor frontLeft = null;
  public DCMotor frontRight = null;
  public DCMotor rearLeft = null;
  public DCMotor rearRight = null;

  public DCMotor intake = null;
  public DCMotor leftRiser = null;
  public DCMotor rightRiser = null;
  public Servo trapdoor = null;
  public Servo leftElbow = null;
  public Servo rightElbow = null;

  public Robot(HardwareMap hardwareMap) {
    frontLeft = new DCMotor(hardwareMap.get(DcMotorEx.class, "FrontLeft"));
    frontRight = new DCMotor(hardwareMap.get(DcMotorEx.class, "FrontRight"), DcMotor.Direction.REVERSE);
    rearLeft = new DCMotor(hardwareMap.get(DcMotorEx.class, "RearLeft"));
    rearRight = new DCMotor(hardwareMap.get(DcMotorEx.class, "RearRight"));

    intake = new DCMotor(hardwareMap.get(DcMotorEx.class, "Intake"), DcMotor.Direction.REVERSE);
    leftRiser = new DCMotor(hardwareMap.get(DcMotorEx.class, "LeftRiser"), DcMotor.Direction.REVERSE);
    rightRiser = new DCMotor(hardwareMap.get(DcMotorEx.class, "RightRiser"));
    trapdoor = hardwareMap.get(Servo.class, "Trapdoor");
    leftElbow = hardwareMap.get(Servo.class, "LeftElbow");
    rightElbow = hardwareMap.get(Servo.class, "RightElbow");
    leftElbow.setDirection(Servo.Direction.REVERSE);
    leftElbow.scaleRange(0.03, 0.4);
    rightElbow.scaleRange(0.03, 0.4);
  }
}