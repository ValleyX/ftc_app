package org.firstinspires.ftc.teamcode.TeamCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Pushbot: AutonomousCaleb", group="Pushbot")
//@Disabled
public class AutonomousCaleb extends LinearOpMode {
    private DcMotor motorLeft;
    private DcMotor motorRight;

    private DigitalChannel digitalTouch;

    private ElapsedTime runtime;

    static final double COUNTS_PER_MOTOR_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 40.0;
    static final double WHEEL_DIAMETER_INCHES = 3.9;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double PERCENT_FULL_CIRCLE_RATIO = (100 / 360); //100 percent divided by 360 degrees

    @Override
    public void runOpMode() throws InterruptedException {
        motorLeft = hardwareMap.dcMotor.get("lmotor");
        motorRight = hardwareMap.dcMotor.get("rmotor");

        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        System.out.println("ValleyX: Waiting for Start");
        waitForStart();

        System.out.println("ValleyX: Starting ....");

        encoderDrive(0.7, 12, 12, 5); //moveboth wheels 12 inches at 70% speed

        //Start motors
        motorLeft.setPower(0.6); //1.0 is full power, 0.0 is no power
        motorRight.setPower(0.6);

        //display digital touch state to phone
        while (digitalTouch.getState() == true) {
            telemetry.addData("Digital Touch", "Is Not Pressed");
        }
        motorLeft.setPower(0.0); //1.0 is full power, 0.0 is no power
        motorRight.setPower(0.0);

        encoderDrive(0.7, -12, -12, 5); //moveboth wheels 12 inches at 70% speed


        int counter = 0;
        while (counter < 5) {
            System.out.printf("Counter %d/n", counter);
            counter = counter + 1;
        }

    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
    }
}

