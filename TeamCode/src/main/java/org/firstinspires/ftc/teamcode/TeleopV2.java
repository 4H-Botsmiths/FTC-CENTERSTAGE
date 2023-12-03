package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.classes.DCMotor;
import org.firstinspires.ftc.teamcode.classes.HDMotor;
import org.firstinspires.ftc.teamcode.classes.CoreMotor;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop
 * period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class Basic instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two
 * wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code
 * folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver
 * Station OpMode list
 */

@TeleOp(name = "Teleop V2", group = "A")
// @Disabled
public class TeleopV2 extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private HDMotor frontLeft = null;
    private HDMotor frontRight = null;
    private HDMotor rearLeft = null;
    private HDMotor rearRight = null;

    private DCMotor intake = null;
    private DCMotor leftRiser = null;
    private DCMotor rightRiser = null;
    private Servo trapdoor = null;
    private Servo leftElbow = null;
    private Servo rightElbow = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        // Initialize the hardware variables. Note that the strings used here as
        // parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeft = new HDMotor(hardwareMap.get(DcMotorEx.class, "FrontLeft"));
        frontRight = new HDMotor(hardwareMap.get(DcMotorEx.class, "FrontRight"), DcMotor.Direction.REVERSE);
        rearLeft = new HDMotor(hardwareMap.get(DcMotorEx.class, "RearLeft"));
        rearRight = new HDMotor(hardwareMap.get(DcMotorEx.class, "RearRight"));

        intake = new DCMotor(hardwareMap.get(DcMotorEx.class, "Intake"), DcMotor.Direction.REVERSE);
        leftRiser = new DCMotor(hardwareMap.get(DcMotorEx.class, "LeftRiser"), DcMotor.Direction.REVERSE);
        rightRiser = new DCMotor(hardwareMap.get(DcMotorEx.class, "RightRiser"));
        trapdoor = hardwareMap.get(Servo.class, "Trapdoor");
        leftElbow = hardwareMap.get(Servo.class, "LeftElbow");
        rightElbow = hardwareMap.get(Servo.class, "RightElbow");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        /*
         * rearRight.setDirection(DcMotor.Direction.REVERSE);
         * rearLeft.setDirection(DcMotor.Direction.REVERSE);
         * frontRight.setDirection(DcMotor.Direction.FORWARD);
         * frontLeft.setDirection(DcMotor.Direction.FORWARD);
         */

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    public void driverLoop() {
        telemetry.addData("Front Left", Math.round(frontLeft.getSpeed() * 100));
        telemetry.addData("Front Right", Math.round(frontRight.getSpeed() * 100));
        telemetry.addData("Rear Right", Math.round(rearRight.getSpeed() * 100));
        telemetry.addData("Rear Left", Math.round(rearLeft.getSpeed() * 100));

        double x = 0.625 * gamepad1.left_stick_x + 0.375 * gamepad1.left_stick_x * gamepad1.right_trigger
                - 0.375 * gamepad1.left_stick_x * gamepad1.left_trigger;
        double y = 0.625 * -gamepad1.left_stick_y + 0.375 * -gamepad1.left_stick_y * gamepad1.right_trigger
                - 0.375 * -gamepad1.left_stick_y * gamepad1.left_trigger;
        double z = 0.625 * gamepad1.right_stick_x + 0.375 * gamepad1.right_stick_x * gamepad1.right_trigger
                - 0.375 * gamepad1.right_stick_x * gamepad1.left_trigger;

        // Drive System
        Drive(x, y, z);
    }

    public void operatorLoop() {
        telemetry.addData("Intake", Math.round(rearLeft.getSpeed() * 100));
        telemetry.addData("Trapdoor", Math.round(trapdoor.getPosition() * 100));
        telemetry.addData("Lift", "Left: %d; Right: %d", Math.round(leftRiser.getSpeed() * 100),
                Math.round(rightRiser.getSpeed() * 100));
        telemetry.addData("Elbow", "Left: %d; Right: %d", Math.round(leftElbow.getPosition() * 100),
                Math.round(rightElbow.getPosition() * 100));

        /*
         * NOTES:
         * what needs to be controlled:
         * > Intake - left trigger DONE
         * > Trapdoor - a DONE
         * > Lift - left joystick DONE
         * > Elbow - right joystick
         */
        if (gamepad2.a) {
            trapdoor.setPosition(0);
            intake.setSpeed(0.2);
        } else {
            trapdoor.setPosition(1);

            if (gamepad2.left_bumper) {
                intake.setSpeed(-gamepad2.left_trigger);
            } else {
                intake.setSpeed(gamepad2.left_trigger);
            }
        }
        leftRiser.setSpeed(-gamepad2.left_stick_y);
        rightRiser.setSpeed(gamepad2.left_stick_y);

        /** 0 = down; 1 = up */
        double elbowPosition = 0;
        leftElbow.setPosition(elbowPosition);
        rightElbow.setPosition(-elbowPosition);
        elbowPosition += -gamepad2.right_stick_y * 0.1;
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Running");

        driverLoop();
        operatorLoop();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    public double m1, m2, m3, m4;

    public void Drive(double x, double y, double z) {
        // r *= steeringMultiplier;
        m1 = Range.clip(y + x + z, -1, 1);
        m2 = Range.clip(y - x - z, -1, 1);
        m3 = Range.clip(y - x + z, -1, 1);
        m4 = Range.clip(y + x - z, -1, 1);
        // gamepad1.rumble((m1 + m3) / 2, (m2 + m4) / 2,
        // Gamepad.RUMBLE_DURATION_CONTINUOUS);
        /*
         * frontLeft.setVelocity(m1);
         * frontRight.setVelocity(m2);
         * rearLeft.setVelocity(m3);
         * rearRight.setVelocity(m4);
         */
        frontLeft.setSpeed(m1);
        frontRight.setSpeed(m2);
        rearLeft.setSpeed(m3);
        rearRight.setSpeed(m4);

    }
}
