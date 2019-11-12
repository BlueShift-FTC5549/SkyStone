package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous",group = "Autonomous")
public class Autonomous {
    private ElapsedTime runtime = new ElapsedTime();

    private Telemetry telemetry;

    private AutoFourWheelDrive autoFourWheelDrive;

    private void initialize() {
        setTelemetryStatus("Initializing");

        autoFourWheelDrive = new AutoFourWheelDrive();

        setTelemetryStatus("Initialized");
    }

    public void runOpMode() {
        int cubePosition; //1, 2, or 3. Corresponds left to right from the perspective of the lander

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
}
