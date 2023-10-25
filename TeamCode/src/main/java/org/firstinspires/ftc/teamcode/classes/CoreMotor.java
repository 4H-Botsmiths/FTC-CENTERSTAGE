package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

public class CoreMotor {
  public DcMotorEx motor;

  public CoreMotor(DcMotorEx motor) {
    this.motor = motor;
    motor.resetDeviceConfigurationForOpMode();
    motor.setDirection(DcMotor.Direction.FORWARD);
    motor.setPower(0);
    motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  }

  public CoreMotor(DcMotorEx motor, DcMotor.Direction direction) {
    this.motor = motor;
    motor.resetDeviceConfigurationForOpMode();
    motor.setDirection(direction);
    motor.setPower(0);
    motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  }

  private final int TICKS_PER_ROTATION = 720;// 288;

  public void setSpeed(double speed) {
    this.motor.setVelocity(speed * TICKS_PER_ROTATION);
  }

  public double getSpeed() {
    return this.motor.getVelocity() / TICKS_PER_ROTATION;
  }
}