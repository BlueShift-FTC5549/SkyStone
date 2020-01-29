package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "BLocks Blue",group = "Autonomous")
public class Blocks_Blue extends LinearOpMode {
    private AutoFourWheelDrive autoFourWheelDrive;
    private int block_position;
    private DistanceSensor distance_sensor;
    private ColorSensor color_sensor;
    private DcMotor raiseSweeper;
    private Servo flipper_servo;
    private Servo flipper_servo2;
    private Servo lift_servo;
    private double counter = 0;

    private void initialize() {
        setTelemetryStatus("Initializing");

        autoFourWheelDrive = new AutoFourWheelDrive(this,"color_sensor","motorDriveLeft","motorDriveRight","imu");
        distance_sensor = this.hardwareMap.get(DistanceSensor.class, "color_sensor");
        color_sensor = this.hardwareMap.colorSensor.get("color_sensor");
        raiseSweeper = this.hardwareMap.get(DcMotor.class,"raiseSweeper");
        flipper_servo = hardwareMap.get(Servo.class,"flipper_servo");
        flipper_servo2 = hardwareMap.get(Servo.class,"flipper_servo2");
        lift_servo = hardwareMap.get(Servo.class,"lift_servo");

        raiseSweeper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        raiseSweeper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        setTelemetryStatus("Initialized");
        flipper_servo.setPosition(0.4);
        flipper_servo2.setPosition(0.7);
        lift_servo.setPosition(.1);
        sleep_sec(.4);
    }
    @Override
    public void runOpMode() {
        initialize();

        waitForStart();

        telemetry.clearAll();
        //Turn the rest of the angle to be facing away from the lander
        autoFourWheelDrive.encoderStrafe(-28,.3);
        while (distance_sensor.getDistance(DistanceUnit.CM) > 5.5) {
            autoFourWheelDrive.strafe(-.1);
            telemetry.addData("Distance",distance_sensor.getDistance(DistanceUnit.CM));
        }
        autoFourWheelDrive.setAllPower(0);
        autoFourWheelDrive.encoderDrive(3,.3);
        block_position = autoFourWheelDrive.find_block();
        telemetry.addData("Block Pos",block_position);
        telemetry.update();
        autoFourWheelDrive.encoderDrive(-2,.3);
        flipper_servo.setPosition(1);
        autoFourWheelDrive.encoderStrafe(6,.5);
        flipper_servo.setPosition(0.83);
        autoFourWheelDrive.encoderDrive(-60.8+block_position*5.4 ,.3);
        //flipper_servo.setPosition(1);
        //sleep_sec(.5);
        //telemetry.addData("Block Position", block_position);
    }

    public void setTelemetryStatus(String status) {
        telemetry.addData("Status", status);
        telemetry.update();
    }

    public AutoFourWheelDrive getAutoFourWheelDrive() {
        return autoFourWheelDrive;
    }

    private void lower_sweeper () {
        while (raiseSweeper.getCurrentPosition() < 14000) {
            raiseSweeper.setPower(1);
            telemetry.addData("Position",raiseSweeper.getCurrentPosition());
            telemetry.update();
        }
        raiseSweeper.setPower(0);
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