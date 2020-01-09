package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous",group = "Autonomous")
public class Autonomous extends LinearOpMode {
    private AutoFourWheelDrive autoFourWheelDrive;
    private int block_position;
    private DistanceSensor distance_sensor;
    private ColorSensor color_sensor;
    private DcMotor raiseSweeper;

    private void initialize() {
        setTelemetryStatus("Initializing");

        autoFourWheelDrive = new AutoFourWheelDrive(this,"color_sensor","motorDriveLeft","motorDriveRight","imu");
        distance_sensor = this.hardwareMap.get(DistanceSensor.class, "color_sensor");
        color_sensor = this.hardwareMap.colorSensor.get("color_sensor");
        raiseSweeper = this.hardwareMap.get(DcMotor.class,"raiseSweeper");

        raiseSweeper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        raiseSweeper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        setTelemetryStatus("Initialized");
    }
    @Override
    public void runOpMode() {
        initialize();

        waitForStart();

        telemetry.clearAll();
        //Turn the rest of the angle to be facing away from the lander
        autoFourWheelDrive.encoderDrive(20);
        autoFourWheelDrive.turn(90);
        block_position = autoFourWheelDrive.find_block();
        telemetry.addData("Color", color_sensor.red());
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
}