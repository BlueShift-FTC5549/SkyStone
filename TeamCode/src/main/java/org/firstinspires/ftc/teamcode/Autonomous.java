package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous",group = "Autonomous")
public class Autonomous extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private AutoFourWheelDrive autoFourWheelDrive;

    private void initialize() {
        setTelemetryStatus("Initializing");

        autoFourWheelDrive = new AutoFourWheelDrive(this, "motorDriveLeft","motorDriveRight","imu",false);

        setTelemetryStatus("Initialized");
    }
    @Override
    public void runOpMode() {
        initialize();

        telemetry.clearAll();

        //Turn the rest of the angle to be facing away from the lander
        autoFourWheelDrive.turn(45, 6);

        autoFourWheelDrive.encoderDrive(10,10);

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
