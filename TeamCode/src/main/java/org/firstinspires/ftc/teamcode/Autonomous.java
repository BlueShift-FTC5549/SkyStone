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

    @Override
    public void runOpMode() {
        int cubePosition; //1, 2, or 3. Corresponds left to right from the perspective of the lander

        initialize();

        waitForStart();

        telemetry.clearAll();

        //Free the lift hook from the lander
        autoFourWheelDrive.encoderStrafe(-5.0, 10);

        //Turn the rest of the angle to be facing away from the lander
        autoFourWheelDrive.turn(45, 6);

        telemetry.addData("Cube Position: ", cubePosition);


        autoFourWheelDrive.encoderStrafe(-15,10);
        telemetry.addData("Status","Turning");

        //Find the gold block
        telemetry.addData("Status","Aligning");
    }

    public void setTelemetryStatus(String status) {
        telemetry.addData("Status", status);
        telemetry.update();
    }
}
