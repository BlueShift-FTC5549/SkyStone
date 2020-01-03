package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous",group = "Autonomous")
public class Autonomous extends LinearOpMode {
    private AutoFourWheelDrive autoFourWheelDrive;
    private int block_position;
    private DistanceSensor distance_sensor;
    private ColorSensor color_sensor;

    private void initialize() {
        setTelemetryStatus("Initializing");

        autoFourWheelDrive = new AutoFourWheelDrive(this,"color_sensor","motorDriveLeft","motorDriveRight",false);
        distance_sensor = this.hardwareMap.get(DistanceSensor.class, "color_sensor");
        color_sensor = this.hardwareMap.colorSensor.get("color_sensor");

        setTelemetryStatus("Initialized");
    }
    @Override
    public void runOpMode() {
        initialize();

        waitForStart();

        telemetry.clearAll();
        autoFourWheelDrive.encoderDrive(50,10);
        //Turn the rest of the angle to be facing away from the lander
        //autoFourWheelDrive.encoderDrive(-71,10);
        //block_position = autoFourWheelDrive.find_block();
        telemetry.addData("Distance",distance_sensor.getDistance(DistanceUnit.CM));
        //while (distance_sensor.getDistance(DistanceUnit.CM) > 7 || (color_sensor.red() <= 20)) {
            //autoFourWheelDrive.setAllPower(-0.5);
            //telemetry.addData("Distance",distance_sensor.getDistance(DistanceUnit.CM));
            //telemetry.addData("Color",color_sensor.red());
            //telemetry.update();
        //}

        //telemetry.addData("Block Position", block_position);
    }

    public void setTelemetryStatus(String status) {
        telemetry.addData("Status", status);
        telemetry.update();
    }

    public AutoFourWheelDrive getAutoFourWheelDrive() {
        return autoFourWheelDrive;
    }
}