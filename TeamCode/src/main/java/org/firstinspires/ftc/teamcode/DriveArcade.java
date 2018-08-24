package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Drive Arcade", group = "Tutorials")
public class DriveArcade extends LinearOpMode
{
    DcMotor leftMotor, rightMotor;
    float   leftPower, rightPower, xValue, yValue;
    private Servo armServo;

    private static final double ARM_RETRACTED_POSITION = 0.4;
    private static final double ARM_EXTENDED_POSITION = 0.6;

    @Override
    public void runOpMode() throws InterruptedException
    {
        leftMotor = hardwareMap.dcMotor.get("lmotor");
        rightMotor = hardwareMap.dcMotor.get("rmotor");

        leftMotor.setDirection(DcMotor.Direction.REVERSE);

        armServo = hardwareMap.servo.get("servo1");
        armServo.setPosition(ARM_RETRACTED_POSITION);

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        System.out.println("ValleyX: Waiting for Start");
        waitForStart();
        System.out.println("ValleyX: Starting...");

        while (opModeIsActive())
        {

            //yValue = -gamepad1.right_stick_y;
            //xValue = -gamepad1.right_stick_x;
            yValue = -gamepad1.left_stick_y;
            xValue = -gamepad1.left_stick_x;

            leftPower =  yValue - xValue;
            rightPower = yValue + xValue;

            leftMotor.setPower(Range.clip(leftPower, -1.0, 1.0));
            rightMotor.setPower(Range.clip(rightPower, -1.0, 1.0));

            telemetry.addData("Mode", "running");
            telemetry.addData("stick", "  y=" + yValue + "  x=" + xValue);
            telemetry.addData("power", "  left=" + leftPower + "  right=" + rightPower);
            telemetry.update();

            if (gamepad1.a)
            {
                armServo.setPosition(ARM_RETRACTED_POSITION);
                System.out.println("ValleyX: GamePad a");
            }
            if (gamepad1.b)
            {
                armServo.setPosition(ARM_EXTENDED_POSITION);
                System.out.println("ValleyX: GamePad b");
            }


            idle();
        }
    }
}
