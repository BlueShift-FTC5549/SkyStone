package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Foundation Blue Bridge",group = "Autonomous")
public class Foundation_Blue_Bridge extends LinearOpMode {
    private AutoFourWheelDrive autoFourWheelDrive;
    private Servo flipper_servo;
    private Servo flipper_servo2;
    private Servo lift_servo;
    private double counter = 0;

    private void initialize() {
        setTelemetryStatus("Initializing");

        autoFourWheelDrive = new AutoFourWheelDrive(this,"color_sensor","motorDriveLeft","motorDriveRight","imu");
        flipper_servo = hardwareMap.get(Servo.class,"flipper_servo");
        flipper_servo2 = hardwareMap.get(Servo.class,"flipper_servo2");
        lift_servo = hardwareMap.get(Servo.class,"lift_servo");


        setTelemetryStatus("Initialized");
        flipper_servo.setPosition(0.4);
        flipper_servo2.setPosition(.7);
        lift_servo.setPosition(.1);
        sleep_sec(.4);
    }
    @Override
    public void runOpMode() {
        initialize();

        waitForStart();

        telemetry.clearAll();
        autoFourWheelDrive.move_foundation(-1,0);
    }

    public void setTelemetryStatus(String status) {
        telemetry.addData("Status", status);
        telemetry.update();
    }

    public AutoFourWheelDrive getAutoFourWheelDrive() {
        return autoFourWheelDrive;
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