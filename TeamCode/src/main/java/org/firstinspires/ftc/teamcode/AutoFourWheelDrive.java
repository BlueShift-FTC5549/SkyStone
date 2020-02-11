package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.nio.channels.DatagramChannel;
import java.util.ArrayList;
import java.util.List;

public class AutoFourWheelDrive {
    private DcMotor motorDriveLeftBack;
    private DcMotor motorDriveLeftFront;
    private DcMotor motorDriveRightBack;
    private DcMotor motorDriveRightFront;
    private DcMotor raiseSweeper;

    //Servos
    private Servo flipper_servo;
    private Servo flipper_servo2;
    private Servo arm_flipper;
    private CRServo extintion_servo;

    //IMU
    private IMU imu;

    //Important Objects to Manipulate
    private Telemetry telemetry;
    private LinearOpMode opMode;
    private ElapsedTime elapsedTime = new ElapsedTime();

    //Color Sensor
    private ColorSensor color_sensor;
    private DistanceSensor distance_sensor;
    private int block_position;

    //Encoder Constants
    private static final double COUNTS_PER_MOTOR_REV    = 1120;    // Andymark Neverest 40
    private static final double DRIVE_GEAR_REDUCTION    = 1.0;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_INCHES   = 4.0;     // For figuring circumference
    private static final double COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    private static final int    ENCODER_NO_MOVEMENT_THRESHOLD = 12;
    private static final double MINIMUM_SPEED = 0.3;
    private static final double ENCODER_STRAFE_Kp = 0.45;
    private static final double SCALE_FACTOR = 55.0/52.0;

    //Turning Constants
    private static final float TURN_Kp = 0.75f;
    private static final float TURN_ERROR_ALLOWANCE = (float)2; //In Degrees
    private static final float TURN_POWER_OFFSET_STEP = (float)0.015;

    public AutoFourWheelDrive(LinearOpMode opMode,  String ColorSensorName, String motorDriveLeftName, String motorDriveRightName, String IMUName) {
        //Bring in all objects from the OpMode and hardwareMap
        this.color_sensor = opMode.hardwareMap.colorSensor.get(ColorSensorName);
        this.motorDriveLeftBack = opMode.hardwareMap.get(DcMotor.class, motorDriveLeftName + "Back");
        this.motorDriveLeftFront = opMode.hardwareMap.get(DcMotor.class, motorDriveLeftName + "Front");
        this.motorDriveRightBack = opMode.hardwareMap.get(DcMotor.class, motorDriveRightName + "Back");
        this.motorDriveRightFront = opMode.hardwareMap.get(DcMotor.class, motorDriveRightName + "Front");
        this.raiseSweeper = opMode.hardwareMap.get(DcMotor.class, "raiseSweeper");
        this.flipper_servo = opMode.hardwareMap.get(Servo.class, "flipper_servo");
        this.flipper_servo2 = opMode.hardwareMap.get(Servo.class,"flipper_servo2");
        this.distance_sensor = opMode.hardwareMap.get(DistanceSensor.class,"color_sensor");
        this.imu = new IMU(opMode.telemetry, opMode.hardwareMap, IMUName);

        this.telemetry = opMode.telemetry;
        this.opMode = opMode;

        //Motor Calibration (Direction and Zero Power Behavior)
        motorDriveLeftBack.setDirection(DcMotor.Direction.FORWARD);
        motorDriveLeftFront.setDirection(DcMotor.Direction.FORWARD);
        motorDriveRightBack.setDirection(DcMotor.Direction.REVERSE);
        motorDriveRightFront.setDirection(DcMotor.Direction.REVERSE);

        motorDriveLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //TODO: Decide whether to brake or not
        motorDriveLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDriveRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDriveRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetEncoders();
    }

    public void turn(float dTheta) {

        int[] previousEncoders = new int[4];
        float thetaInit = imu.getHeading();
        float thetaTarget = (thetaInit + dTheta) % 360;
        float turningPowerOffset = 0.16f;

        telemetry.addData("Turning", "Starting at " + thetaInit + " degrees");
        telemetry.update();

        resetEncoders();
        elapsedTime.reset();
        while (Math.abs(thetaTarget - imu.getHeading()) % 360.0 > TURN_ERROR_ALLOWANCE
                && opMode.opModeIsActive()
                && !opMode.isStopRequested()) {

            int[] currentEncoders = getEncoderValues();

            double thetaCurrent = (double)imu.getHeading();

            double thetaError = BlueShiftUtil.getDegreeDifference(thetaCurrent, thetaTarget);
            double thetaPercentError = Math.abs(thetaError / dTheta);

            double turnPower = Math.signum(thetaError) * (TURN_Kp * thetaPercentError * (1.0 - thetaPercentError) + turningPowerOffset);

            setTurnPower(turnPower);

            if (!hasAllEncodersMoved(previousEncoders, currentEncoders)) {
                turningPowerOffset += thetaPercentError * TURN_POWER_OFFSET_STEP;
            }

            previousEncoders = currentEncoders;

            telemetry.addData("Theta Error", thetaError);
            telemetry.addData("Percent Error", thetaPercentError);
            telemetry.addData("Current Heading", thetaCurrent);
            telemetry.addData("Target Heading", thetaTarget);
            telemetry.addData("Motor Power", turnPower);
            telemetry.update();
        }

        abortMotion();

        telemetry.addData("Turning", "Complete at " + imu.getHeading() + " degrees");
        telemetry.update();
    }
    public void encoderStrafe(double targetDistance,double maxspeed) {
        resetEncoders();
        int factor = 1;
        if (Math.abs(targetDistance) != targetDistance){
            factor = -1;
        }
        double speed = 0;
        double  encodertarget = Math.abs(targetDistance * COUNTS_PER_INCH)* SCALE_FACTOR;
        while (checkEncoders(encodertarget) && opMode.opModeIsActive() && !opMode.isStopRequested()) {
            if (averageEncoderValue() < encodertarget*ENCODER_STRAFE_Kp){
                speed = factor*(averageEncoderValue()*(1.0-MINIMUM_SPEED)/(encodertarget*ENCODER_STRAFE_Kp) + MINIMUM_SPEED);
            }
            else if (averageEncoderValue() > encodertarget*(1.0-ENCODER_STRAFE_Kp)) {
                speed = factor*((encodertarget-averageEncoderValue()) * (1.0-MINIMUM_SPEED)/(encodertarget*ENCODER_STRAFE_Kp) + MINIMUM_SPEED);
            }
            else if (averageEncoderValue() > encodertarget*ENCODER_STRAFE_Kp
                    && averageEncoderValue() < encodertarget*(1.0-ENCODER_STRAFE_Kp)){
                speed = factor;
            }
            if (speed > maxspeed) {
                speed = maxspeed;
            }
            strafe(speed);
        }
        //Stop the robot and terminate any loops running
        abortMotion();

        telemetry.addData("Encoder Driving", "Complete");
        telemetry.update();
    }

    /**
     * Drive the robot to a certain encoder value on the drive train motors.
     *
     * @param targetDistance The distance in inches to travel
     */
    public void encoderDrive(double targetDistance,double maxspeed) {
        resetEncoders();
        int factor = 1;
        if (Math.abs(targetDistance) != targetDistance){
            factor = -1;
        }
        double speed = 0;
        double encodertarget = Math.abs(targetDistance * COUNTS_PER_INCH)*SCALE_FACTOR;
        telemetry.addData("Original Encoder Target",targetDistance*COUNTS_PER_INCH);
        telemetry.addData("Adjusted Encoder Target",encodertarget);
        telemetry.update();
        while (checkEncoders(encodertarget) && opMode.opModeIsActive() && !opMode.isStopRequested()) {
            if (averageEncoderValue() < encodertarget*ENCODER_STRAFE_Kp){
                speed = factor*(averageEncoderValue()*(1.0-MINIMUM_SPEED)/(encodertarget*ENCODER_STRAFE_Kp) + MINIMUM_SPEED);
            }
            else if (averageEncoderValue() > encodertarget*(1.0-ENCODER_STRAFE_Kp)) {
                speed = factor*((encodertarget-averageEncoderValue()) * (1.0-MINIMUM_SPEED)/(encodertarget*ENCODER_STRAFE_Kp) + MINIMUM_SPEED);
            }
            else if (averageEncoderValue() > encodertarget*ENCODER_STRAFE_Kp
                    && averageEncoderValue() < encodertarget*(1.0-ENCODER_STRAFE_Kp)){
                speed = factor;
            }
            if (speed > maxspeed) speed = maxspeed;
            setAllPower(speed);
        }
        //Stop the robot and terminate any loops running
        abortMotion();

        telemetry.addData("Encoder Driving", "Complete");
        telemetry.update();
    }

    private void resetEncoders() {
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
    }

    private void abortMotion() {
        setAllPower(0);

        telemetry.addData("Status", "Drive Motion Aborted");
        telemetry.update();
    }

    private void setSplitPower(double leftPower, double rightPower) {
        motorDriveLeftBack.setPower(leftPower);
        motorDriveLeftFront.setPower(leftPower);
        motorDriveRightBack.setPower(rightPower);
        motorDriveRightFront.setPower(rightPower);
    }

    /**
     * Set the power of every motor to one power.
     *
     * @param power Desired motor power
     */
    public void setAllPower(double power) {
        motorDriveLeftBack.setPower(power);
        motorDriveLeftFront.setPower(power);
        motorDriveRightBack.setPower(power);
        motorDriveRightFront.setPower(power);
    }

    /**
     * Set the power of every motor to one power except the right side is reversed to turn
     *
     * @param power Desired motor power
     */
    private void setTurnPower(double power) {
        motorDriveLeftBack.setPower(power);
        motorDriveLeftFront.setPower(power);
        motorDriveRightBack.setPower(-power);
        motorDriveRightFront.setPower(-power);
    }

    private int[] getEncoderValues() {
        return new int[] {
                motorDriveLeftBack.getCurrentPosition(),
                motorDriveLeftFront.getCurrentPosition(),
                motorDriveRightBack.getCurrentPosition(),
                motorDriveRightFront.getCurrentPosition()
        };
    }

    private int averageEncoderValue () {
        int [] encoderPositions = getEncoderValues();
        return (Math.abs(encoderPositions[0]) + Math.abs(encoderPositions[1]) + Math.abs(encoderPositions[2]) + Math.abs(encoderPositions[3]))/4;
    }

    /**
     * Compares two arrays of encoder values and determines whether or not all motors haved moved.
     *
     * @param encoders_1 The first array of encoder values
     * @param encoders_2 The second array of encoder values
     * @return Whether or not all motors have moved
     */
    private boolean hasAllEncodersMoved(int[] encoders_1, int[] encoders_2) {
        if (Math.abs(encoders_1[0] - encoders_2[0]) < ENCODER_NO_MOVEMENT_THRESHOLD
                || Math.abs(encoders_1[1] - encoders_2[1]) < ENCODER_NO_MOVEMENT_THRESHOLD
                || Math.abs(encoders_1[2] - encoders_2[2]) < ENCODER_NO_MOVEMENT_THRESHOLD
                || Math.abs(encoders_1[3] - encoders_2[3]) < ENCODER_NO_MOVEMENT_THRESHOLD) {
            return false;
        }

        return true;
    }

    private boolean checkEncoders (double encodervalue) {
        if (Math.abs(motorDriveLeftBack.getCurrentPosition()) < (encodervalue)
            && (Math.abs(motorDriveLeftFront.getCurrentPosition()) < (encodervalue))
            && (Math.abs(motorDriveRightBack.getCurrentPosition()) < (encodervalue))
            && (Math.abs(motorDriveRightFront.getCurrentPosition()) < (encodervalue))) {
            return true;
        }
    return false;
    }

    public void strafe(double power) {
        motorDriveLeftFront.setPower(power);
        motorDriveRightBack.setPower(power);

        motorDriveRightFront.setPower(-power);
        motorDriveLeftBack.setPower(-power);
    }

    public int check_block(int trial,int side) {
        telemetry.addData("Red",color_sensor.red());
        sleep_sec(.5);
        if (color_sensor.red() < 30){
            return trial;
        }
        else {
            encoderDrive(-7*side,.1);
            while (distance_sensor.getDistance(DistanceUnit.CM) > 5.9) {
                strafe(-.1);
                telemetry.addData("Distance",distance_sensor.getDistance(DistanceUnit.CM));
            }
            setAllPower(0);
        }
        return 0;
    }
    public int find_block (int side) {
        block_position = check_block(1,side);
        if (block_position == 0) {
            block_position = check_block(2,side);
        }
        if (block_position == 0) {
            block_position = check_block(3,side);
        }
        /*while (block_position < 10) {
            telemetry.addData("Alpha",color_sensor.alpha());
            telemetry.addData("Red", color_sensor.red());
            telemetry.addData("Green",color_sensor.green());
            telemetry.addData("block position", block_position);
            telemetry.update();
        }*/
        return block_position;
    }
    public void move_foundation (int side, int park_side) {
        // Bring Robot away from depot
        encoderDrive(11*side,.3);
        sleep_sec(.25);

        // Drive Robot to foundation
        encoderStrafe(-29,.3);
        sleep_sec(.25);

        // Put The arms down
        flipper_servo.setPosition(1);
        flipper_servo2.setPosition(0);
        sleep_sec(1);

        // Drive Robot back to wall to get foundation in depot
        encoderStrafe(32,.3);
        sleep_sec(.25);

        // Lift arms back up
        flipper_servo2.setPosition(.4);
        flipper_servo.setPosition(.4);
        sleep_sec(.4);

        // Turn slightly to counter turning while strafing
        // Drive Robot towards parking zone
        encoderDrive(-47*side,.3);
        sleep_sec(.25);

        // Move robot out of the way of other robot
        if (park_side == 0) {
            encoderStrafe(27*side,.2);
        }
    }

    public void block_side (int side) {
        // Drive to blocks
        encoderStrafe(-28, .3);

        // Use Distance sensor to move up
        while (distance_sensor.getDistance(DistanceUnit.CM) > 5.5) {
            strafe(-.1);
            telemetry.addData("Distance", distance_sensor.getDistance(DistanceUnit.CM));
        }
        setAllPower(0);

        // Drive back to make sure sensor senses block
        encoderDrive(3 * side, .3);

        // find SkyStone
        block_position = find_block(side);

        // Drive to center arm
        encoderDrive(-2 * side, .3);

        // flip arms down
        if (side == 1) { flipper_servo.setPosition(1); }
        else if (side == -1) {flipper_servo2.setPosition(0); }

        // drive back
        encoderStrafe(6, .5);

        // bring arm up a little
        if (side == 1) {flipper_servo.setPosition(0.83);}
        else if (side == -1) {flipper_servo2.setPosition(.1);}

        // bring block over to other side
        encoderDrive((-60.8 + block_position * 5.4)*side, .3);

        // flip arms up
        flipper_servo.setPosition(0);
        flipper_servo2.setPosition(.7);

        // park
        encoderDrive(20 * side, .3);

        // drive forward
        encoderStrafe(-4,.1);
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