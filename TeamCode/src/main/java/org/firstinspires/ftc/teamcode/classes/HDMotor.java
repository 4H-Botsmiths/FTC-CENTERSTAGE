package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

public class HDMotor {
  public DcMotorEx motor;

  public HDMotor(DcMotorEx motor) {
    this.motor = motor;
    motor.resetDeviceConfigurationForOpMode();
    motor.setDirection(DcMotor.Direction.FORWARD);
    motor.setPower(0);
    motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  }

  public HDMotor(DcMotorEx motor, DcMotor.Direction direction) {
    this.motor = motor;
    motor.resetDeviceConfigurationForOpMode();
    motor.setDirection(direction);
    motor.setPower(0);
    motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  }

  private final int TICKS_PER_ROTATION = 3124;// 28;
  private final int MAX_VELOCITY = 2000; //Actually higher, but this will ensure consistency

  public void setSpeed(double speed) {
    this.motor.setVelocity(speed * MAX_VELOCITY);
  }

  public double getSpeed() {
    return this.motor.getVelocity() / MAX_VELOCITY;
  }
}