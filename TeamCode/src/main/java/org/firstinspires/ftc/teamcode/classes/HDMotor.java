package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

public class HDMotor {
  public DcMotorEx motor;

  public HDMotor(DcMotorEx motor) {
    this.motor = motor;
  }

  private final int TICKS_PER_ROTATION = 3124;// 28;

  public void setSpeed(double speed) {
    this.motor.setVelocity(speed * TICKS_PER_ROTATION);
  }

  public double getSpeed() {
    return this.motor.getVelocity() / TICKS_PER_ROTATION;
  }

  public void configure() {
    motor.resetDeviceConfigurationForOpMode();
    motor.setDirection(DcMotor.Direction.FORWARD);
    motor.setPower(0);
    motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  }
}