package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Platform Blue",group = "Autonomous")
public class PlatformBlue extends LinearOpMode {
    private AutoFourWheelDrive autoFourWheelDrive;
    private int block_position;
    private DistanceSensor distance_sensor;
    private ColorSensor color_sensor;
    private DcMotor raiseSweeper;
    private Servo flipper_servo;
    private Servo flipper_servo2;
    private double counter = 0;

    private void initialize() {
        setTelemetryStatus("Initializing");

        autoFourWheelDrive = new AutoFourWheelDrive(this,"color_sensor","motorDriveLeft","motorDriveRight","imu");
        distance_sensor = this.hardwareMap.get(DistanceSensor.class, "color_sensor");
        color_sensor = this.hardwareMap.colorSensor.get("color_sensor");
        raiseSweeper = this.hardwareMap.get(DcMotor.class,"raiseSweeper");
        flipper_servo = hardwareMap.get(Servo.class,"flipper_servo");
        flipper_servo2 = hardwareMap.get(Servo.class,"flipper_servo2");

        raiseSweeper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        raiseSweeper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        setTelemetryStatus("Initialized");
        flipper_servo.setPosition(0.4);
        flipper_servo2.setPosition(.9);
        sleep_sec(.4);
    }
    @Override
    public void runOpMode() {
        initialize();

        waitForStart();

        telemetry.clearAll();
        //Turn the rest of the angle to be facing away from the lander
        //telemetry.addData("Color", color_sensor.red());
        autoFourWheelDrive.encoderDrive(-13,.5);
        sleep_sec(.25);
        autoFourWheelDrive.encoderStrafe(-29,.3);
        sleep_sec(.25);
        flipper_servo.setPosition(1);
        flipper_servo2.setPosition(0.3);
        sleep_sec(1);
        autoFourWheelDrive.encoderStrafe(-2,0.1);
        sleep_sec(.5);
        autoFourWheelDrive.encoderStrafe(30.3,.7);
        sleep_sec(.25);
        flipper_servo2.setPosition(.9);
        flipper_servo.setPosition(.5);
        sleep_sec(.4);
        autoFourWheelDrive.turn(5);
        sleep_sec(.25);
        autoFourWheelDrive.encoderDrive(29,1);
        sleep_sec(.25);
        autoFourWheelDrive.turn(-90);
        autoFourWheelDrive.encoderStrafe(-20,.7);
        sleep_sec(.25);
        autoFourWheelDrive.encoderDrive(1.5,.1);
        telemetry.addData("Block Pos",block_position);
        telemetry.update();

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