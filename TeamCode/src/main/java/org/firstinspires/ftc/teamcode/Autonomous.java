package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous",group = "Autonomous")
public class Autonomous extends LinearOpMode {
    private AutoFourWheelDrive autoFourWheelDrive;
    private int block_position;

    private void initialize() {
        setTelemetryStatus("Initializing");

        autoFourWheelDrive = new AutoFourWheelDrive(this,"color_sensor","motorDriveLeft","motorDriveRight",false);

        setTelemetryStatus("Initialized");
    }
    @Override
    public void runOpMode() {
        initialize();

        waitForStart();

        telemetry.clearAll();

        //Turn the rest of the angle to be facing away from the lander
        autoFourWheelDrive.encoderDrive(-70,10);
        block_position = autoFourWheelDrive.find_block();

        setTelemetryStatus("Turning");
    }

    public void setTelemetryStatus(String status) {
        telemetry.addData("Status", status);
        telemetry.update();
    }

    public AutoFourWheelDrive getAutoFourWheelDrive() {
        return autoFourWheelDrive;
    }
}