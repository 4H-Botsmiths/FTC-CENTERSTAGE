package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.classes.DCMotor;

public class Robot {
  /** Port 2.0 ? */
  public DCMotor frontLeft = null;
  /** Port 1 */
  public DCMotor frontRight = null;
  /** Port 2 */
  public DCMotor rearLeft = null;
  /** Port 3 */
  public DCMotor rearRight = null;

  /** Port 0 */
  public DCMotor intake = null;
  /** Port 2.1 ? */
  public DCMotor leftRiser = null;
  /** Port 2.2 ? */
  public DCMotor rightRiser = null;
  /** Port 0 */
  public Servo trapdoor = null;
  /** Port 1 ?  */
  public Servo leftElbow = null;
  /** Port 2 ? */
  public Servo rightElbow = null;

  public Robot(HardwareMap hardwareMap) {
    frontLeft = new DCMotor(hardwareMap.get(DcMotorEx.class, "FrontLeft"));
    frontRight = new DCMotor(hardwareMap.get(DcMotorEx.class, "FrontRight"), DcMotor.Direction.REVERSE);
    rearLeft = new DCMotor(hardwareMap.get(DcMotorEx.class, "RearLeft"));
    rearRight = new DCMotor(hardwareMap.get(DcMotorEx.class, "RearRight"));

    intake = new DCMotor(hardwareMap.get(DcMotorEx.class, "Intake"), DcMotor.Direction.REVERSE);
    leftRiser = new DCMotor(hardwareMap.get(DcMotorEx.class, "LeftRiser"));
    rightRiser = new DCMotor(hardwareMap.get(DcMotorEx.class, "RightRiser"), DcMotor.Direction.REVERSE);
    trapdoor = hardwareMap.get(Servo.class, "Trapdoor");
    leftElbow = hardwareMap.get(Servo.class, "LeftElbow");
    rightElbow = hardwareMap.get(Servo.class, "RightElbow");
    leftElbow.setDirection(Servo.Direction.REVERSE);
    leftElbow.scaleRange(0.03, 0.4);
    rightElbow.scaleRange(0.03, 0.4);
  }
}