package org.firstinspires.ftc.teamcode.TeamCode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    private AnalogInput topPot;

    private Servo hangingServo;
    private Servo lockServo;
    private RevBlinkinLedDriver led;


    double topMax = 1.946;
    double topClosed = 0.001;
    double bottomClosed = 0.6;
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime gameruntime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException
    {
        System.out.println("ValleyX runOpMode");

        motorLeft = hardwareMap.dcMotor.get("lmotor"); // main 1 motor
        motorRight = hardwareMap.dcMotor.get("rmotor"); // main 0 motor
        bottomLift = hardwareMap.dcMotor.get("blift"); // main 2 motor
        bottomLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topLift = hardwareMap.dcMotor.get("tlift"); // main 3 motor
        topLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.dcMotor.get("intake"); // secondary 0 motor --> fix in wiring
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        digitalTouch = hardwareMap.get(DigitalChannel.class, "touch"); // secondary 0 digital device
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        led = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        bottomPot = hardwareMap.analogInput.get("bottomPot"); // main 0 analog input
        topPot = hardwareMap.analogInput.get("topPot"); // main 2 analog input

        hangingServo = hardwareMap.servo.get("hservo"); // main 0 servo
        lockServo = hardwareMap.servo.get("lockServo"); // secondary 0 servo


        motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        double gamepad1LeftStickY; // motor left
        double gamepad1RightStickY; // motor right
        double gamepad2LeftStickY;
        double gamepad2RightStickY;
        double gamepad1LeftTrigger;

        System.out.println("ValleyX waiting for start");
        waitForStart();
        gameruntime.reset();
        System.out.println("ValleyX started");

        while (opModeIsActive())
        {
            gamepad1LeftStickY = gamepad1.left_stick_y;
            gamepad1RightStickY = gamepad1.right_stick_y;
            gamepad2LeftStickY = gamepad2.left_stick_y;
            gamepad2RightStickY = gamepad2.right_stick_y;
            gamepad1LeftTrigger = gamepad1.left_trigger;

            telemetry.addData("Status", "Run Time: " + gameruntime.toString());
            telemetry.addData("gp1", "  leftY=" + gamepad1LeftStickY + "  rightY=" + gamepad1RightStickY);
            telemetry.addData("gp2", "  leftY=" + gamepad2LeftStickY + "  rightY=" + gamepad2RightStickY);

            if (gameruntime.seconds() > 115)
            {
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            }
            else if (gameruntime.seconds() > 106)
            {
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            }
            else if (gameruntime.seconds() > 96)
            {
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }
            else if (gameruntime.seconds() > 91)
            {
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
            }

            // gamepad1
            motorLeft.setPower(-gamepad1RightStickY);
            motorRight.setPower(-gamepad1LeftStickY);

            if (gamepad1.x == true) // true means pressed
            {
                // hanging servo opening
                hangingServo.setPosition(1.0);
                lockServo.setPosition(0.0);
            }
            if (gamepad1.y == true)
            {
                // hanging servo closing
                hangingServo.setPosition(0.0);
            }

            if (gamepad1.a == true) // pressed
            {
                // move bottom lift to hanging pos
                System.out.println("ValleyX A button pressed");
                motorLeft.setPower(0.0);
                motorRight.setPower(0.0);

                // close top
                goToPosition(topLift, topPot, topClosed, 0.6);
                /*
                while (topPot.getVoltage() > 0.35)
                {
                    topLift.setPower(-0.6);
                }
                topLift.setPower(0.0);
*/
                // open up to get to hanging
                goToPosition(bottomLift, bottomPot, 1.345, -0.6);
                /*
                while (bottomPot.getVoltage() < 1.39)
                {
                    bottomLift.setPower(-0.6);
                }
                bottomLift.setPower(0.0);

                // closing to get to hanging
                while (bottomPot.getVoltage() > 1.48)
                {
                    bottomLift.setPower(0.6);
                }
                bottomLift.setPower(0.0);
                */
            }


/*
            if (gamepad1.b == true) //pressed
            {
                // move bottom lift to zero/starting pos
                System.out.println("ValleyX B button pressed");
                motorLeft.setPower(0.0);
                motorRight.setPower(0.0);

                lockServo.setPosition(0.0);
                // closing top
                goToPosition(topLift, topPot, topClosed, 0.6);

                while (topPot.getVoltage() > 0.35)
                {
                    topLift.setPower(-0.6);
                }

                //topLift.setPower(0.0);
                //topPot.resetDeviceConfigurationForOpMode(); // reset topPot to zero

                // closing bottom
                goToPosition(bottomLift, bottomPot, bottomClosed, -0.6);

                while (digitalTouch.getState() == true) // true means not pressed
                {
                    bottomLift.setPower(0.6);
                }
                bottomLift.setPower(0.0);
                bottomPot.resetDeviceConfigurationForOpMode(); // reset bottomPot to zero

            }*/


            if (gamepad1.left_bumper == true) //true means pressed
            {
                lockServo.setPosition(1.0);
            }




            //gamepad2
            if (gamepad2.x == true) // pressed
            {
                if (gameruntime.seconds() < 91)
                {
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE);
                }
                // move bottom lift to intake pos
                System.out.println("ValleyX X button pressed");
                motorLeft.setPower(0.0);
                motorRight.setPower(0.0);
                //goToPosition(topLift, topPot, 1.97, 0.6); //////////////////////////mark this///
                goToPosition(topLift, topPot, 1.2, 0.6);
                goToPosition(bottomLift, bottomPot, 3.3, -0.6);
                if (gameruntime.seconds() < 91)
                {
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_END_TO_END_BLEND_1_TO_2);
                }
            }
            if (gamepad2.a == true) // pressed
            {
                if (gameruntime.seconds() < 91)
                {
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                }
                // set bottom pot to drive back to landing position
                goToPosition(bottomLift, bottomPot, 1.2, -0.6); //1.75
                goToPosition(topLift, topPot, 1.46, 0.6); //2.44
                goToPosition(bottomLift, bottomPot, 1.2, -0.6);
                // bottom to DBL position --> open
                if (gameruntime.seconds() < 91)
                {
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_END_TO_END_BLEND_1_TO_2);
                }
            }

            if (gamepad2.b == true) // pressed
            {
                if (gameruntime.seconds() < 91)
                {
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE);
                }
                // move bottom lift to unloading pos
                System.out.println("ValleyX B button pressed");
                motorLeft.setPower(0.0);
                motorRight.setPower(0.0);

                goToPosition(bottomLift, bottomPot, 1.2, -0.6);
                goToPosition(topLift, topPot, 1.04, 0.6); //1.0
                if (gameruntime.seconds() < 91)
                {
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_END_TO_END_BLEND_1_TO_2);
                }
            }

            if (gamepad2.right_bumper == true) // motor for intake --> in
            {
                intake.setPower(-10.0); // as long as the bumper is held down
            }
            else if (gamepad2.left_bumper == true) // motor for intake --> out
            {
                intake.setPower(10.0); // as long as the bumper is held down
            }
            else
            {
                intake.setPower(0.0);
            }

            if ((gamepad2LeftStickY < 0.0) && (bottomPot.getVoltage() < bottomClosed))
            {
                bottomLift.setPower(0.0);
            }
            else
            {
                bottomLift.setPower(-gamepad2LeftStickY);
            }

            if ((gamepad2RightStickY < 0.0) && (topPot.getVoltage() < topClosed))
            {
                topLift.setPower(0.0);
            }
            else
            {
                topLift.setPower(gamepad2RightStickY);
            }

            telemetry.addData("bottomPot values ", bottomPot.getVoltage());
            telemetry.addData("topPot values ", topPot.getVoltage());

            telemetry.addData("servo values", hangingServo.getPosition());

            telemetry.addData("trigger values", gamepad1LeftTrigger);
            telemetry.addData("touch", digitalTouch.getState());
            telemetry.update();
            idle();
        } // while opmode is active
    } // run op mode

    private void goToPosition(DcMotor lift, AnalogInput pot, double position, double power)
    {
        double timeoutS = 4.0;
        runtime.reset();
        if (pot.getVoltage() < position)
        {
            lift.setPower(power);
            while ((pot.getVoltage() < position) && (runtime.seconds() < timeoutS) && opModeIsActive() && (gamepad2.dpad_down == false)) // false = not pressed
            {
                // gamepad1
                motorLeft.setPower(-gamepad1.right_stick_y);
                motorRight.setPower(-gamepad1.left_stick_y);

                checkForIntake();
                idle();
            }
            lift.setPower(0.0);

        }
        else
        {
            lift.setPower(-power);
            while ((pot.getVoltage() > position) && (runtime.seconds() < timeoutS) && opModeIsActive() && (gamepad2.dpad_down == false)) // false = not pressed
            {
                // gamepad1
                motorLeft.setPower(-gamepad1.right_stick_y);
                motorRight.setPower(-gamepad1.left_stick_y);
                checkForIntake();

                idle();
            }
            lift.setPower(0.0);
        }
    }

    private void checkForIntake()
    {
        if (gamepad2.right_bumper == true) // motor for intake --> in
        {
            intake.setPower(-10.0); // as long as the bumper is held down
        } else if (gamepad2.left_bumper == true) // motor for intake --> out
        {
            intake.setPower(10.0); // as long as the bumper is held down
        } else {
            intake.setPower(0.0);
        }
    }

} // class
