package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.robotcore.hardware.DcMotor;

public class DCMotor {
  public DcMotor motor;

  public DCMotor(DcMotor motor) {
    this.motor = motor;
    motor.resetDeviceConfigurationForOpMode();
    motor.setDirection(DcMotor.Direction.FORWARD);
    motor.setPower(0);
    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  }

  public DCMotor(DcMotor motor, DcMotor.Direction direction) {
    this.motor = motor;
    motor.resetDeviceConfigurationForOpMode();
    motor.setDirection(direction);
    motor.setPower(0);
    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  }

  /**
   * Alias for {@link #DcMotor.setPower()}
   * 
   * @param speed the speed
   */
  public void setSpeed(double speed) {
    this.motor.setPower(speed);
  }

  /**
   * Alias for {@link #DcMotor.getPower()}
   * 
   * @param speed the speed
   */
  public double getSpeed() {
    return this.motor.getPower();
  }
}