package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

public class AutoFourWheelDrive {
    private DcMotor motorDriveLeftBack;
    private DcMotor motorDriveLeftFront;
    private DcMotor motorDriveRightBack;
    private DcMotor motorDriveRightFront;

    //Important Objects to Manipulate
    private Telemetry telemetry;
    private LinearOpMode opMode;
    private ElapsedTime elapsedTime = new ElapsedTime();

    //IMU
    private IMU imu;

    //Color Sensor
    private ColorSensor color_sensor;

    //Instance Variables
    private boolean hasAborted;
    private boolean verboseLoops;
    private List<Float> headingStorage;

    //Turning Constants
    private static final float TURN_Kp = 0.75f;
    private static final float TURN_ERROR_ALLOWANCE = (float)5; //In Degrees
    private static final float TURN_POWER_OFFSET_STEP = (float)0.015;

    //Encoder Constants
    private static final double COUNTS_PER_MOTOR_REV    = 1120;    // Andymark Neverest 40
    private static final double DRIVE_GEAR_REDUCTION    = 24.0/32.0;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_INCHES   = 4.0;     // For figuring circumference
    private static final double COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    private static final float  ENCODER_DRIVE_Kp        = 0.85f;
    private static final int    ENCODER_DRIVE_ERROR_ALLOWANCE = 200;
    private static final float  ENCODER_DRIVE_POWER_OFFSET_STEP = (float)0.013;
    private static final int    ENCODER_NO_MOVEMENT_THRESHOLD = 12;

    public AutoFourWheelDrive(LinearOpMode opMode, String ColorSensorName, String motorDriveLeftName, String motorDriveRightName, String IMUName, boolean verboseLoops) {
        //Bring in all objects from the OpMode and hardwareMap
        this.color_sensor = opMode.hardwareMap.colorSensor.get(ColorSensorName);
        this.motorDriveLeftBack = opMode.hardwareMap.get(DcMotor.class, motorDriveLeftName + "Back");
        this.motorDriveLeftFront = opMode.hardwareMap.get(DcMotor.class, motorDriveLeftName + "Front");
        this.motorDriveRightBack = opMode.hardwareMap.get(DcMotor.class, motorDriveRightName + "Back");
        this.motorDriveRightFront = opMode.hardwareMap.get(DcMotor.class, motorDriveRightName + "Front");
        this.imu = new IMU(opMode.telemetry, opMode.hardwareMap, IMUName);

        this.telemetry = opMode.telemetry;
        this.opMode = opMode;

        this.hasAborted = false;
        this.verboseLoops = verboseLoops;
        this.headingStorage = new ArrayList<>();


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

    public void turn(float dTheta, double secondsTimeout) {
        hasAborted = false;

        int[] previousEncoders = new int[4];
        float thetaInit = imu.getHeading();
        float thetaTarget = (thetaInit + dTheta) % 360;
        float turningPowerOffset = 0.16f;

        telemetry.addData("Turning", "Starting at " + thetaInit + " degrees");
        telemetry.update();

        resetEncoders();
        elapsedTime.reset();

        if (!verboseLoops) {
            while (Math.abs(thetaTarget - imu.getHeading()) % 360.0 > TURN_ERROR_ALLOWANCE
                    && opMode.opModeIsActive()
                    && elapsedTime.seconds() <= secondsTimeout
                    && !hasAborted
                    && !opMode.isStopRequested()) {

                int[] currentEncoders = getEncoderValues();

                double thetaCurrent = (double)imu.getHeading();

                double thetaError = BlueShiftUtil.getDegreeDifference(thetaCurrent, thetaTarget);
                double thetaPercentError = Math.abs(thetaError / dTheta);

                double turnPower = Math.signum(thetaError) * (TURN_Kp * thetaPercentError * (1.0 - thetaPercentError) + turningPowerOffset);

                setTurnPower(turnPower);

                if (!hasAllEncodersMoved(previousEncoders, currentEncoders)) {
                    turningPowerOffset += thetaPercentError * TURN_POWER_OFFSET_STEP; //TODO: Decide whether or not to make offset proportional
                }

                previousEncoders = currentEncoders;
            }
        } else {
            while (Math.abs(thetaTarget - imu.getHeading()) % 360.0 > TURN_ERROR_ALLOWANCE
                    && opMode.opModeIsActive()
                    && elapsedTime.seconds() <= secondsTimeout
                    && !hasAborted
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
        }

        abortMotion();

        telemetry.addData("Turning", "Complete at " + imu.getHeading() + " degrees");
        telemetry.update();
    }
    public void encoderStrafe(double targetDistance, double secondsTimeout) {
        hasAborted = false;

        //Reset encoder values to zero
        resetEncoders();

        //Find desired encoder value and initialize placeholder array
        int[] currentEncoders = getEncoderValues();
        int[] previousEncoders = new int[4];
        int encoderTarget = (int)(targetDistance * COUNTS_PER_INCH);
        double ENCODER_DRIVE_POWER_OFFSET = 0.18;

        //Telemetry Information
        telemetry.addData("Encoder Driving", "Starting at %7d : %7d", motorDriveLeftBack.getCurrentPosition(), motorDriveRightBack.getCurrentPosition());
        telemetry.update();

        //Set the current time to zero
        elapsedTime.reset();

        if (!verboseLoops) {
            while (elapsedTime.seconds() <= secondsTimeout
                    && (Math.abs(BlueShiftUtil.modifiedMeanAbsoluteError(currentEncoders, encoderTarget)) - encoderTarget) > ENCODER_DRIVE_ERROR_ALLOWANCE
                    && opMode.opModeIsActive()
                    && !hasAborted
                    && !opMode.isStopRequested()) {

                currentEncoders = getEncoderValues();

                double encoderMeanAbsoluteError = BlueShiftUtil.modifiedMeanAbsoluteError(currentEncoders, encoderTarget);
                float percentEncoderError = (float) Range.clip(Math.abs((float)encoderMeanAbsoluteError / (float)encoderTarget), 0.0, 1.0);

                double motorPower = Math.signum(encoderMeanAbsoluteError) * (ENCODER_DRIVE_Kp * percentEncoderError * (1 - percentEncoderError) + ENCODER_DRIVE_POWER_OFFSET);

                strafe(motorPower);

                if (!hasAllEncodersMoved(previousEncoders, currentEncoders)) {
                    ENCODER_DRIVE_POWER_OFFSET += ENCODER_DRIVE_POWER_OFFSET_STEP;
                }

                previousEncoders = currentEncoders;
            }
        } else {
            while (elapsedTime.seconds() <= secondsTimeout
                    && Math.abs(BlueShiftUtil.modifiedMeanAbsoluteError(currentEncoders, encoderTarget)) > ENCODER_DRIVE_ERROR_ALLOWANCE
                    && opMode.opModeIsActive()
                    && !hasAborted
                    && !opMode.isStopRequested()) {

                currentEncoders = getEncoderValues();

                double encoderMeanAbsoluteError = BlueShiftUtil.modifiedMeanAbsoluteError(currentEncoders, encoderTarget);
                float percentEncoderError = (float)Range.clip(Math.abs((float)encoderMeanAbsoluteError / (float)encoderTarget), 0.0, 1.0);

                double motorPower = Math.signum(encoderMeanAbsoluteError) * (ENCODER_DRIVE_Kp * percentEncoderError * (1 - percentEncoderError) + ENCODER_DRIVE_POWER_OFFSET);

                strafe(motorPower);

                if (!hasAllEncodersMoved(previousEncoders, currentEncoders)) {
                    ENCODER_DRIVE_POWER_OFFSET += ENCODER_DRIVE_POWER_OFFSET_STEP;
                }

                previousEncoders = currentEncoders;

                telemetry.addData("Front Encoders", "(%7d):(%7d)", motorDriveLeftFront.getCurrentPosition(), motorDriveRightFront.getCurrentPosition());
                telemetry.addData("Back Encoders", "(%7d):(%7d)", motorDriveLeftBack.getCurrentPosition(), motorDriveRightBack.getCurrentPosition());
                telemetry.addLine()
                        .addData("Encoder Target ", encoderTarget + "\n")
                        .addData("Encoder MAE", encoderMeanAbsoluteError + "\n")
                        .addData("Percent Errors", percentEncoderError + "\n")
                        .addData("Power", motorPower + "\n");
                telemetry.update();
            }
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
     * @param secondsTimeout The maximum seconds to run the loop
     */
    public void encoderDrive(double targetDistance, double secondsTimeout) {
        hasAborted = false;

        //Reset encoder values to zero
        resetEncoders();

        //Find desired encoder value and initialize placeholder array
        int[] previousEncoders = new int[4];
        int encoderTarget = (int)(targetDistance * COUNTS_PER_INCH);
        double ENCODER_DRIVE_POWER_OFFSET = 0.15;

        //Telemetry Information
        telemetry.addData("Encoder Driving", "Starting at %7d : %7d", motorDriveLeftBack.getCurrentPosition(), motorDriveRightBack.getCurrentPosition());
        telemetry.update();

        //Set the current time to zero
        elapsedTime.reset();

        if (!verboseLoops) {
            while (elapsedTime.seconds() <= secondsTimeout
                    && (Math.abs(motorDriveLeftBack.getCurrentPosition() - encoderTarget) > ENCODER_DRIVE_ERROR_ALLOWANCE
                    || Math.abs(motorDriveRightBack.getCurrentPosition() - encoderTarget) > ENCODER_DRIVE_ERROR_ALLOWANCE)
                    && opMode.opModeIsActive()
                    && !hasAborted
                    && !opMode.isStopRequested()) {

                int[] currentEncoders = getEncoderValues();

                int motorDriveLeftEncoderError = encoderTarget - motorDriveLeftBack.getCurrentPosition();
                int motorDriveRightEncoderError = encoderTarget - motorDriveRightBack.getCurrentPosition();
                double motorDriveLeftPercentEncoderError = (double)(motorDriveLeftEncoderError) / (double)encoderTarget;
                double motorDriveRightPercentEncoderError = (double)(motorDriveRightEncoderError) / (double)encoderTarget;

                double motorDriveLeftPower = Math.signum(motorDriveLeftEncoderError) * (ENCODER_DRIVE_Kp * Math.abs(motorDriveLeftPercentEncoderError) * (1 - Math.abs(motorDriveLeftPercentEncoderError)) + ENCODER_DRIVE_POWER_OFFSET);
                double motorDriveRightPower = Math.signum(motorDriveRightEncoderError) * (ENCODER_DRIVE_Kp * Math.abs(motorDriveRightPercentEncoderError) * (1 - Math.abs(motorDriveRightPercentEncoderError)) + ENCODER_DRIVE_POWER_OFFSET);

                setSplitPower(motorDriveLeftPower, motorDriveRightPower);

                if (!hasAllEncodersMoved(previousEncoders, currentEncoders)) {
                    ENCODER_DRIVE_POWER_OFFSET += (motorDriveLeftPercentEncoderError + motorDriveRightPercentEncoderError)/2.0 * ENCODER_DRIVE_POWER_OFFSET_STEP;
                }

                previousEncoders = currentEncoders;
            }
        } else {
            while (elapsedTime.seconds() <= secondsTimeout
                    && (Math.abs(motorDriveLeftBack.getCurrentPosition() - encoderTarget) > ENCODER_DRIVE_ERROR_ALLOWANCE
                    || Math.abs(motorDriveRightBack.getCurrentPosition() - encoderTarget) > ENCODER_DRIVE_ERROR_ALLOWANCE)
                    && opMode.opModeIsActive()
                    && !hasAborted
                    && !opMode.isStopRequested()) {

                int[] currentEncoders = getEncoderValues();

                int motorDriveLeftEncoderError = encoderTarget - motorDriveLeftBack.getCurrentPosition();
                int motorDriveRightEncoderError = encoderTarget - motorDriveRightBack.getCurrentPosition();
                double motorDriveLeftPercentEncoderError = (double)(motorDriveLeftEncoderError) / (double)encoderTarget;
                double motorDriveRightPercentEncoderError = (double)(motorDriveRightEncoderError) / (double)encoderTarget;

                double motorDriveLeftPower = Math.signum(motorDriveLeftEncoderError) * (ENCODER_DRIVE_Kp * Math.abs(motorDriveLeftPercentEncoderError) * (1 - Math.abs(motorDriveLeftPercentEncoderError)) + ENCODER_DRIVE_POWER_OFFSET);
                double motorDriveRightPower = Math.signum(motorDriveRightEncoderError) * (ENCODER_DRIVE_Kp * Math.abs(motorDriveRightPercentEncoderError) * (1 - Math.abs(motorDriveRightPercentEncoderError)) + ENCODER_DRIVE_POWER_OFFSET);

                setSplitPower(motorDriveLeftPower, motorDriveRightPower);

                if (!hasAllEncodersMoved(previousEncoders, currentEncoders)) {
                    ENCODER_DRIVE_POWER_OFFSET += (motorDriveLeftPercentEncoderError + motorDriveRightPercentEncoderError)/2.0 * ENCODER_DRIVE_POWER_OFFSET_STEP;
                }

                previousEncoders = currentEncoders;

                telemetry.addData("Boolean checks", (Math.abs(motorDriveLeftBack.getCurrentPosition() - encoderTarget) > ENCODER_DRIVE_ERROR_ALLOWANCE
                        || Math.abs(motorDriveRightBack.getCurrentPosition() - encoderTarget) > ENCODER_DRIVE_ERROR_ALLOWANCE));
                telemetry.addData("Front Encoders", "(%7d):(%7d)", motorDriveLeftFront.getCurrentPosition(), motorDriveRightFront.getCurrentPosition());
                telemetry.addData("Back Encoders", "(%7d):(%7d)", motorDriveLeftBack.getCurrentPosition(), motorDriveRightBack.getCurrentPosition());
                telemetry.addData("Encoder Target", encoderTarget);
                telemetry.addData("Encoder Errors", "(%7d):(%7d)", motorDriveLeftEncoderError, motorDriveRightEncoderError);
                telemetry.addData("Percent Errors", "(%.2f):(%.2f)", motorDriveLeftPercentEncoderError, motorDriveRightPercentEncoderError);
                telemetry.addData("Power", "Left (%.2f), Right (%.2f)", motorDriveLeftPower, motorDriveRightPower);
                telemetry.addData("Timing", "(%.2f) of (%.2f)", elapsedTime.seconds(), secondsTimeout);
                telemetry.update();
            }
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

        motorDriveLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDriveLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDriveRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDriveRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void abortMotion() {
        hasAborted = true;

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
    private void setAllPower(double power) {
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

    public void strafe(double power) {
        motorDriveLeftFront.setPower(power);
        motorDriveRightBack.setPower(power);

        motorDriveRightFront.setPower(-power);
        motorDriveLeftBack.setPower(-power);
    }

    public void color_value () {
        telemetry.addData("Value:",color_sensor.alpha());
    }
}