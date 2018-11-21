package org.firstinspires.ftc.teamcode.TeamCode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Pushbot: AutonomousJon", group="Pushbot")
@Disabled
public class AutonomousJon extends LinearOpMode
{
    private DcMotor motorLeft;
    private DcMotor motorRight;
    private DigitalChannel digitalTouch;
    private ElapsedTime runtime;
    static final double COUNTS_PER_MOTOR_REV  = 28;
    static final double DRIVE_GEAR_REDUCTION = 40.0;
    static final double WHEEL_DIAMETER_INCHES = 3.9;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);


    private GoldAlignDetector detector;

    @Override
    public void runOpMode() throws InterruptedException
    {
       // motorLeft = hardwareMap.dcMotor.get("lmotor");
        //motorRight = hardwareMap.dcMotor.get("rmotor");
        //digitalTouch = hardwareMap.get(DigitalChannel.class, "touch");
        //ElapsedTime runtime = new ElapsedTime();
        GoldAlignDetector goldAlignDetector = new GoldAlignDetector();

        //motorLeft.setDirection(DcMotor.Direction.REVERSE);
        //digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        System.out.println("ValleyX: Waiting for Start");
        waitForStart();

        System.out.println("ValleyX: Starting .... ");

        System.out.println("ValleyX: Looking for gold");


        telemetry.addData("Status", "DogeCV 2018.0 - Gold Align Example");


        detector = new GoldAlignDetector();

        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());

        detector.useDefaults();


        // Optional Tuning

        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)

        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.

        detector.downscale = 0.4; // How much to downscale the input frames


        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA

        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring

        detector.maxAreaScorer.weight = 0.005;
        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;
        detector.enable();

        while (opModeIsActive())
        {
            telemetry.addData("IsAligned", detector.getAligned()); // Is the bot aligned with the gold mineral
            telemetry.addData("X Pos", detector.getXPosition()); // Gold X pos in the view 270 - 370 is aligned
            telemetry.addData("IsFound", detector.isFound()); //is the gold in view
            telemetry.update();
        }

        System.out.println("ValleyX: ending");
        detector.disable();

/*

        while (goldAlignDetector.isFound() == false)
        {
            if (goldAlignDetector.getAligned() == true)
            {
                System.out.printf("ValleyX: Gold in View x position = %f\n", goldAlignDetector.getXPosition());
            }
        }
        System.out.printf("ValleyX: Gold straight ahead\n");
*/
/*
        encoderDrive(0.7, 12,12,5); //move both wheels 12 inches at 70% speed

        //Start motors
        motorLeft.setPower(0.6);  //1.0 is full power, 0.0 is no power
        motorRight.setPower(0.6);

        //display digital touch state to phone
        if (digitalTouch.getState() == true)
        {
            telemetry.addData("Digital Touch", "Is Not Pressed");
        }
        else
        {
            telemetry.addData("Digital Touch", "Is Pressed");
        }

        int counter = 0;
        while (counter < 5)
        {
           System.out.printf("Counter %d\n", counter);
           counter = counter + 1;
        }
        */
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = motorLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = motorRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            motorLeft.setTargetPosition(newLeftTarget);
            motorRight.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            motorLeft.setPower(Math.abs(speed));
            motorRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorLeft.isBusy() && motorRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        motorLeft.getCurrentPosition(),
                        motorRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            motorLeft.setPower(0);
            motorRight.setPower(0);

            // Turn off RUN_TO_POSITION
            motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(1);   // optional pause after each move
        }
    }


}
