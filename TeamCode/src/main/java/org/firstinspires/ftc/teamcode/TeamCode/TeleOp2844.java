package org.firstinspires.ftc.teamcode.TeamCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOp 2844", group = "TeamCode")
public class TeleOp2844 extends LinearOpMode
{
    private DcMotor motorLeft;
    private DcMotor motorRight;
    private DcMotor bottomLift;
    private DcMotor topLift;
    private DcMotor intake;
    private DigitalChannel digitalTouch;
    private AnalogInput bottomPot;
    //private AnalogInput topPot; // additional potentiameter to be added on top lift
    private Servo hangingServo;



    @Override
    public void runOpMode() throws InterruptedException
    {
        System.out.println("ValleyX runOpMode");

        motorLeft = hardwareMap.dcMotor.get("lmotor"); // main 1 motor
        motorRight = hardwareMap.dcMotor.get("rmotor"); // main 0 motor
        bottomLift = hardwareMap.dcMotor.get("blift"); // main 2 motor
        topLift = hardwareMap.dcMotor.get("tlift"); // main 3 motor
        intake = hardwareMap.dcMotor.get("intake"); // secondary 0 motor
        digitalTouch = hardwareMap.get(DigitalChannel.class, "touch"); // secondary 0 digital device
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);
        bottomPot = hardwareMap.analogInput.get("bottomPot"); // main 0 analog input
        //topPot = hardwareMap.analogInput.get("topPot"); // not installed yet
        hangingServo = hardwareMap.servo.get("hservo"); // main 0 servo


        motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        double gamepad1LeftStickY;
        double gamepad1RightStickY;
        double gamepad2LeftStickY;
        double gamepad2RightStickY;
        double gamepad1LeftTrigger;

        waitForStart();

        System.out.println("ValleyX started");

        while (opModeIsActive())
        {
            gamepad1LeftStickY = gamepad1.left_stick_y;
            gamepad1RightStickY = gamepad1.right_stick_y;
            gamepad2LeftStickY = gamepad2.left_stick_y;
            gamepad2RightStickY = gamepad2.right_stick_y;
            gamepad1LeftTrigger = gamepad1.left_trigger;

            telemetry.addData("gp1", "  leftY=" + gamepad1LeftStickY + "  rightY=" + gamepad1RightStickY);

            motorLeft.setPower(gamepad1LeftStickY);
            motorRight.setPower(gamepad1RightStickY);

            if (gamepad2.y == true) //pressed
            {
                // move bottom lift to zero/starting pos
                System.out.println("ValleyX Y button pressed");
                motorLeft.setPower(0.0);
                motorRight.setPower(0.0);
                while (digitalTouch.getState() == true) // true means not pressed
                {
                    bottomLift.setPower(-0.6);
                }
                bottomLift.setPower(0.0);
                bottomPot.resetDeviceConfigurationForOpMode(); // reset bottomPot to zero
            }

            if (gamepad2.x == true) // pressed
            {
                // move bottom lift to intake pos
                System.out.println("ValleyX X button pressed");
                motorLeft.setPower(0.0);
                motorRight.setPower(0.0);
                while (bottomPot.getVoltage() < 2.793)
                {
                    bottomLift.setPower(0.6);
                }
                bottomLift.setPower(0.0);
            }

            if (gamepad2.a == true) // pressed
            {
                // move bottom lift to hanging pos
                System.out.println("ValleyX A button pressed");
                motorLeft.setPower(0.0);
                motorRight.setPower(0.0);
                while (bottomPot.getVoltage() < 1.167)
                {
                    bottomLift.setPower(0.6);
                }
                bottomLift.setPower(0.0);
            }

            if (gamepad2.b == true) // pressed
            {
                // move bottom lift to unloading pos
                System.out.println("ValleyX B button pressed");
                motorLeft.setPower(0.0);
                motorRight.setPower(0.0);
                while (bottomPot.getVoltage() < 1.0)
                {
                    bottomLift.setPower(0.6);
                }
                bottomLift.setPower(0.0);

                idle();
            }

            if (gamepad2.left_bumper == true) // motor for intake --> foam pieces
            {
                intake.setPower(0.6); // as long as the bumper is held down
            }
            if (gamepad2.right_bumper == true) // motor for intake --> foam pieces
            {
                intake.setPower(-0.6); // as long as the bumper is held down
            }
            else
            {
                intake.setPower(0.0);
            }
            if (gamepad1.x == true) // true means pressed
            {
                // hanging servo opening
                hangingServo.setPosition(1.0);
            }
            if (gamepad1.y == true)
            {
                // hanging servo closing
                hangingServo.setPosition(0.0);
            }
            bottomLift.setPower(-gamepad2LeftStickY);
            topLift.setPower(gamepad2RightStickY);
            //telemetry.addData("topPot values ", topPot.getVoltage());
            telemetry.addData("bottomPot values ", bottomPot.getVoltage());
            telemetry.addData("servo values", hangingServo.getPosition());
            telemetry.addData("trigger values", gamepad1LeftTrigger);
            telemetry.addData("touch", digitalTouch.getState());
            telemetry.update();
        } // while opmode is active
    } // run op mode
} // class
