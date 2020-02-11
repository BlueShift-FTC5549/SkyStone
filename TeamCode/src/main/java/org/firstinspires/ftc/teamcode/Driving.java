package org.firstinspires.ftc.teamcode;

import android.os.CountDownTimer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Drivin",group = "TeleOp")
public class Driving extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor one;
    private DcMotor two;
    private DcMotor three;
    private DcMotor four;
    private CountDownTimer timer;

    private void initialize() {
        setTelemetryStatus("Initializing");
        one = this.hardwareMap.get(DcMotor.class,  "one");
        two = this.hardwareMap.get(DcMotor.class, "two");
        three = this.hardwareMap.get(DcMotor.class,  "three");
        four = this.hardwareMap.get(DcMotor.class, "four");
        setTelemetryStatus("Initialized");
    }

    public void runOpMode() {
        initialize();

        if (gamepad1.right_stick_x != 0) {
            // right side
            one.setPower(gamepad1.right_stick_x);
            three.setPower(gamepad1.right_stick_x);

            // left side
            two.setPower(-gamepad1.right_stick_x);
            four.setPower(-gamepad1.right_stick_x);
        }

        if (gamepad1.right_stick_y != 0) {
            one.setPower(gamepad1.right_stick_y);
            three.setPower(gamepad1.right_stick_y);
            two.setPower(gamepad1.right_stick_y);
            four.setPower(gamepad1.right_stick_y);
        }

        telemetry.clearAll();


    }


    public void setTelemetryStatus(String status) {
        telemetry.addData("Status", status);
        telemetry.update();
    }
}
