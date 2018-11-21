package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOp Tutorial", group = "Tutorials")
@Disabled
public class TeleOpTutorial extends LinearOpMode
{
    private DcMotor motorLeft;
    private DcMotor motorRight;

    private Servo armServo;

    private static final double ARM_RETRACTED_POSITION = 0.2;
    private static final double ARM_EXTENDED_POSITION = 0.8;


    @Override
    public void runOpMode() throws InterruptedException
    {
        motorLeft = hardwareMap.dcMotor.get("lmotor");
        motorRight = hardwareMap.dcMotor.get("rmotor");

        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        armServo = hardwareMap.servo.get("servo1");
        armServo.setPosition(ARM_RETRACTED_POSITION);

        Log.d("Jon","Wait for start");

        waitForStart();

        while (opModeIsActive())
        {
            //string k = " k" + gamepad1.left_stick_y;
            if (gamepad1.left_stick_y != 0.0) {
                Log.d("Jon", "left stick power: " + gamepad1.left_stick_y);
            }

            motorLeft.setPower(-gamepad1.left_stick_y);
            motorRight.setPower(-gamepad1.right_stick_y);

            if (gamepad1.a)
            {
                armServo.setPosition(ARM_RETRACTED_POSITION);
            }
            if (gamepad1.b)
            {
                armServo.setPosition(ARM_EXTENDED_POSITION);
            }

            idle();
        }
    }
}
