package org.firstinspires.ftc.teamcode.TeamCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOp Samantha", group = "Tutorials")
public class SamanthaTeleOp extends LinearOpMode
{
    private DcMotor motorLeft;
    private DcMotor motorRight;

    //private Servo clawServo;
    //private Servo armServo;

    private static final double CLAW_FORWARD_POSITION = 0.8;
    private static final double CLAW_BACKWARD_POSITION = 0.2;

    private static final double ARM_EXTENDED_POSITION = 0.8;
    private static final double ARM_RETRACTED_POSIION = 0.2;

    @Override
    public void runOpMode() throws InterruptedException
    {
        motorLeft = hardwareMap.dcMotor.get("lmotor");
        motorRight = hardwareMap.dcMotor.get("rmotor");

        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        //clawServo = hardwareMap.servo.get("servo1");
        //armServo = hardwareMap.servo.get("servo0");
        //armServo.setPosition(ARM_RETRACTED_POSIION);
        double gamepad1LeftStickY;
        double gamepad1RightStickY;

        waitForStart();

        while (opModeIsActive())
        {
            gamepad1LeftStickY = gamepad1.left_stick_y;
            gamepad1RightStickY = gamepad1.right_stick_y;

            //telemetry.addData("Mode", "running");
            telemetry.addData("gp1", "  leftY=" + gamepad1LeftStickY + "  rightY=" + gamepad1RightStickY);
            //telemetry.addData("power", "  left=" + leftPower + "  right=" + rightPower);
            telemetry.update();

            motorLeft.setPower(-gamepad1LeftStickY);
            motorRight.setPower(-gamepad1RightStickY);

            /*if (gamepad1.a)
            {
                clawServo.setPosition(CLAW_FORWARD_POSITION);
            }
            if (gamepad1.b)
            {
                clawServo.setPosition(CLAW_BACKWARD_POSITION);
            }

            if (gamepad1.x)
            {
                armServo.setPosition(ARM_EXTENDED_POSITION);
            }
            if (gamepad1.y)
            {
                armServo.setPosition(ARM_RETRACTED_POSIION);
            } */



            idle();
        }

    }
}
