package org.firstinspires.ftc.teamcode.TeamCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Pushbot: AutonomousSamantha", group="Pushbot")
public class AutonomousSamantha extends LinearOpMode
{
    HardwareTest robot = new HardwareTest();
    private ElapsedTime     runtime = new ElapsedTime();

   //private DcMotor motorLeft;
    //private DcMotor motorRight;
    private DigitalChannel digitalTouch;
    //private ElapsedTime runtime;
    static final double COUNTS_PER_MOTOR_REV  = 28;
    static final double DRIVE_GEAR_REDUCTION = 40.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double RobotDiameter = 11.3;
    static final double CIRCUMFERENCE = RobotDiameter * 3.1415;

    public double degToInches (double degrees)
    {
        return (((1.0/360.0)* degrees) * CIRCUMFERENCE);
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot.init (hardwareMap);

        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //motorLeft = hardwareMap.dcMotor.get("lmotor");
        //motorRight = hardwareMap.dcMotor.get("rmotor");
        digitalTouch = hardwareMap.get(DigitalChannel.class, "touch");

        //motorLeft.setDirection(DcMotor.Direction.REVERSE);
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);
        System.out.printf("ValleyX: Turning in %f\n", degToInches(90));

        System.out.println("ValleyX: Waiting for Start");
        waitForStart();

        System.out.println("ValleyX: Starting .... ");
        /*
        encoderDrive(0.3, degToInches(360), -degToInches(360), 5); //move both wheels 12 in at 30% speed
        encoderDrive(0.3, -degToInches(360), degToInches(360), 5); //move both wheels 12 in at 30% speed


        encoderDrive(0.3, 12, 12, 5); //move both wheels 12 in at 30% speed
        encoderDrive(0.3, -12, -12, 5); //move both wheels 12 in at 30% speed
        encoderDrive(0.3, 12, 12, 5); //move both wheels 12 in at 30% speed
        encoderDrive(0.3, -12, -12, 5); //move both wheels 12 in at 30% speed
        encoderDrive(0.3, degToInches(90), -degToInches(90), 5); //move both wheels 12 in at 30% speed
        encoderDrive(0.3, -degToInches(90), degToInches(90), 5); //move both wheels 12 in at 30% speed
*/


        while (true) //loop forever
        {
            if (digitalTouch.getState() == true) //true means not pressed
            {
                telemetry.addData("Digital Touch", "Is Not Pressed");
                robot.leftDrive.setPower(0.3); //1.0 is full power, 0.0 is no power
                robot.rightDrive.setPower(0.3);
                idle(); //wait for motors to respond
            }
            else
            {
                System.out.println("ValleyX: Touch Pressed");
                telemetry.addData("Digital Touch", "Is Pressed");
                robot.leftDrive.setPower(0.0); //1.0 is full power, 0.0 is no power
                robot.rightDrive.setPower(0.0);

                System.out.println("ValleyX: Backing Up 12 Inches");
                encoderDrive(0.7, -12, -12, 5); //move both wheels 12 in at 70% speed
                System.out.println("ValleyX: Turning Left 90 Degrees");
                System.out.printf("ValleyX: Turning in %f\n", degToInches(90));
                encoderDrive(0.7, degToInches(90), -degToInches(90), 5); //move both wheels 12 in at 70% speed
                robot.leftDrive.setPower(0.3); //1.0 is full power, 0.0 is no power
                robot.rightDrive.setPower(0.3);
                idle(); //wait for motors to respond
            }
        }

    }
    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS)
    {
        int newLeftTarget;
        int newRightTarget;

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Ensure that the opmode is still active
        if (opModeIsActive())
        {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftDrive.setPower(Math.abs(speed));
            robot.rightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftDrive.isBusy() && robot.rightDrive.isBusy()))
            {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftDrive.getCurrentPosition(),
                       robot.rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(1);   // optional pause after each move
        }
    }
}
