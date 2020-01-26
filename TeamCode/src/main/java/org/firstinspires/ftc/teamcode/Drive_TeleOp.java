/* Copyright (c) 2018 Blue Shift Robotics
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOP", group="Main")
public class Drive_TeleOp extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorDriveLeftBack;
    private DcMotor motorDriveLeftFront;
    private DcMotor motorDriveRightBack;
    private DcMotor motorDriveRightFront;
    private DcMotor raiseSweeper;
    private DcMotor sweeperRight;
    private DcMotor sweeperLeft;
    private Servo flipper_servo;
    private Servo flipper_servo2;

    private boolean slow = false;
    private double slow_value = 5;

    @Override public void init() {
        telemetry.clearAll();
        telemetry.addData("Status", "TeleOP Initialization In Progress");
        telemetry.update();

        // Retrieve the motor objects from the hardware map. These names come from the configuration in the robot controller.
        motorDriveLeftBack = hardwareMap.get(DcMotor.class,   "motorDriveLeftBack");
        motorDriveLeftFront = hardwareMap.get(DcMotor.class,  "motorDriveLeftFront");
        motorDriveRightBack = hardwareMap.get(DcMotor.class,  "motorDriveRightBack");
        motorDriveRightFront = hardwareMap.get(DcMotor.class, "motorDriveRightFront");
        raiseSweeper = hardwareMap.get(DcMotor.class,  "raiseSweeper");
        sweeperRight = hardwareMap.get(DcMotor.class,  "sweeperRight");
        sweeperLeft = hardwareMap.get(DcMotor.class, "sweeperLeft");
        flipper_servo = hardwareMap.get(Servo.class, "flipper_servo");
        flipper_servo2 = hardwareMap.get(Servo.class,"flipper_servo2");
        //flip_server = hardwareMap.get(Servo.class, "flip_servo");

        // Since one motor is reversed in relation to the other, we must reverse the motor on the right so positive powers mean forward.
        motorDriveLeftBack.setDirection(DcMotor.Direction.REVERSE);
        motorDriveLeftFront.setDirection(DcMotor.Direction.REVERSE);
        motorDriveRightBack.setDirection(DcMotor.Direction.FORWARD);
        motorDriveRightFront.setDirection(DcMotor.Direction.FORWARD);

        motorDriveLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        raiseSweeper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorDriveLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDriveLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDriveRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDriveRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        raiseSweeper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        flipper_servo2.setPosition(.9);
        flipper_servo.setPosition(.4);

    }

    @Override public void init_loop() { }

    @Override public void start() {
        runtime.reset();
        telemetry.clearAll();

        telemetry.addData("Status", "Started");
    }

    @Override public void loop() {
        if (gamepad1.y) {
            slow = !slow;
        }

        if (slow) {
            drive(slow_value);
        }
        else {
            drive(1);
        }

        if (gamepad1.right_trigger > 0){
            sweeperRight.setPower(gamepad1.right_trigger);
            sweeperLeft.setPower(-gamepad1.right_trigger);
        }
        else if (gamepad1.left_trigger > 0) {
            sweeperRight.setPower(-gamepad1.left_trigger/3);
            sweeperLeft.setPower(gamepad1.left_trigger/3);
        }
        else if (gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0) {
            sweeperLeft.setPower(0);
            sweeperRight.setPower(0);
        }
        if (gamepad1.right_bumper) {
            raiseSweeper.setPower(-1);
        }
        else if (gamepad1.left_bumper) {
            raiseSweeper.setPower(1);
        }
        else if (!gamepad1.right_bumper && !gamepad1.left_bumper) {
            raiseSweeper.setPower(0);
        }

        if (gamepad1.dpad_down) {
            flipper_servo2.setPosition(.3);
            flipper_servo.setPosition(1);
            sleep_sec(.7);
        }
        else if (gamepad1.dpad_up) {
            flipper_servo.setPosition(.4);
            flipper_servo2.setPosition(.9);
            sleep_sec(.7);
        }

        telemetry.addData("Raise Sweeper Position",raiseSweeper.getCurrentPosition());
        telemetry.addData("Right X",gamepad1.right_stick_x);
        telemetry.addData("Right Y",gamepad1.right_stick_y);
        telemetry.addData("Left X",gamepad1.left_stick_x);
        telemetry.addData("Left Y",gamepad1.left_stick_y);
        telemetry.update();
    }
    private void drive (double power_factor) {
        //Driving Code
        double speed = Math.sqrt(2) * Math.pow(Math.pow(gamepad1.left_stick_y, 4) + Math.pow(-gamepad1.left_stick_x, 4), 0.5);
        double angle = Math.atan2(-gamepad1.left_stick_x, gamepad1.left_stick_y);

        float primaryDiagonalSpeed = (float) (speed * Math.sin(angle + (Math.PI / 4.0)));
        float secondaryDiagonalSpeed = (float) (speed * Math.cos(angle + (Math.PI / 4.0)));

        motorDriveLeftBack.setPower(secondaryDiagonalSpeed/power_factor);
        motorDriveRightFront.setPower(secondaryDiagonalSpeed/power_factor);
        motorDriveLeftFront.setPower(primaryDiagonalSpeed/power_factor);
        motorDriveRightBack.setPower(primaryDiagonalSpeed/power_factor);

        setSplitPower(gamepad1.right_stick_x/power_factor);

    }

    public void setSplitPower(double power) {
        motorDriveLeftBack.setPower(power);
        motorDriveLeftFront.setPower(power);
        motorDriveRightBack.setPower(-power);
        motorDriveRightFront.setPower(-power);
    }

    public void strafe(double power) {
        motorDriveLeftFront.setPower(power);
        motorDriveRightBack.setPower(power);
        motorDriveRightFront.setPower(-power);
        motorDriveLeftBack.setPower(-power);
    }

    public void setAllPower (double power){
        motorDriveLeftBack.setPower(-power);
        motorDriveLeftFront.setPower(power);
        motorDriveRightBack.setPower(-power);
        motorDriveRightFront.setPower(power);
    }

    private void sleep_sec (double time_seconds) {
        try
        {
            Thread.sleep((int)time_seconds*1000);
        }
        catch(InterruptedException ex)
        {
            Thread.currentThread().interrupt();
        }
    }
}