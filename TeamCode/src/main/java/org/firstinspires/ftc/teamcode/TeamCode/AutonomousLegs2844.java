package org.firstinspires.ftc.teamcode.TeamCode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="Robot: AutonomousLegs2844", group="TeamCode")
public class AutonomousLegs2844 extends LinearOpMode
{
    private ElapsedTime runtime = new ElapsedTime();

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30;

    private GoldAlignDetector detector;

    private DcMotor motorLeft;
    private DcMotor motorRight;


    static final double COUNTS_PER_MOTOR_REV  = 28;   //hw spec for rev motor encoder
    static final double DRIVE_GEAR_REDUCTION = 40.0;  //gear reduction for wheel motor
    static final double WHEEL_DIAMETER_INCHES = 4.0;  //wheel diameter
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415); //calculate number of encoder counts per inch

    //parms to use encoders for rotating at angles
    static final double RobotDiameter = 11.3; // diameter of robot between wheels
    static final double CIRCUMFERENCE = RobotDiameter * 3.1415; // circumference of circle formed when robot makes 360 turn

    enum FoundRotationLocation // to store where the gold cube was found
    {
        LEFT,
        STRAIGHT,
        RIGHT
    } ;

    FoundRotationLocation foundRot = FoundRotationLocation.STRAIGHT; // default to straight

    // initial aligned x and y dimentions
    int goldDetectorLeftX = 260;
    int goldDetectorRightX = 320;

    // final aligned x and y dementions --> moved over because of phone position
    static final int goldIsFoundLeftX = 70;
    static final int goldIsFoundRightX = 530;

    //converts degrees to inches for the given bot
    public double degToInches (double degrees)
    {
        return (((1.0/360.0)* degrees) * CIRCUMFERENCE);

    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        motorLeft = hardwareMap.dcMotor.get("lmotor"); // main 1 motor
        motorRight = hardwareMap.dcMotor.get("rmotor"); // main 0 motor

        motorLeft.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        motorRight.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors


        detector = new GoldAlignDetector();

        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());

        detector.useDefaults();

        detector.alignSize = 200;

        detector.alignPosOffset = 0;

        detector.downscale = 0.4;

        detector.SetRequestedYLine(310); //enhancement to doge detector to only consider scoring
                                            //matches >= the Y line

        //detector.areaScoringMethod = DogeCV.AreaScoringMethod.PERFECT_AREA;
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;

        detector.maxAreaScorer.weight = 0.005;
        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();

        // Set all motors to zero power
        motorLeft.setPower(0);
        motorRight.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters(); // inertial motion unit

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        /* ---new remapping code --*/
        /*
        //swapping y & z axis due to vertical mounting of rev expansion board

        //byte AXIS_MAP_CONFIG_BYTE = 0x6; //This is what to write to the AXIS_MAP_CONFIG register to swap x and z axes
        byte AXIS_MAP_CONFIG_BYTE = 0x18; //This is what to write to the AXIS_MAP_CONFIG register to swap y and z axes
        byte AXIS_MAP_SIGN_BYTE = 0x1; //This is what to write to the AXIS_MAP_SIGN register to negate the z axis

        //Need to be in CONFIG mode to write to registers
        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.CONFIG.bVal);

        sleep(100); //Changing modes requires a delay before doing anything else

        //Write to the AXIS_MAP_CONFIG register
        imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG,AXIS_MAP_CONFIG_BYTE);

        //Write to the AXIS_MAP_SIGN register
        imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN,AXIS_MAP_SIGN_BYTE);

        //Need to change back into the IMU mode to use the gyro
        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.IMU.bVal & 0x0F);

        sleep(100); //Changing modes again requires a delay
        */
        /* ---new remapping code ---*/

        System.out.println("ValleyX: Waiting for Start");

        telemetry.addData("Status", "Ready for Start");
        telemetry.update();

        waitForStart();

        System.out.println("ValleyX: Starting .... ");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "calibrated");
        telemetry.update();


        encoderDrive(0.3, 4, 4, 1);
        int counter = 0; // 0=straight, 1=right, 2=left

        String foundString = "not found"; //default

        int alignCount = 0; // 0=first alignment, 1=second alignment

        while (opModeIsActive())
        {
            //refine isfound to be less than what doge detector says
            boolean isFound = (detector.getXPosition() < goldIsFoundRightX) &&
                    (detector.getXPosition() > goldIsFoundLeftX) &&
                    (detector.isFound() == true);

            telemetry.addData("IsAligned", detector.getAligned()); // Is the bot aligned with the gold mineral
            telemetry.addData("X Pos", detector.getXPosition()); // Gold X pos in the view 270 - 370 is aligned
            telemetry.addData("IsFound", isFound); //is the gold in view
            telemetry.update();

            System.out.println("ValleyXIsAligned " + detector.getAligned()); // Is the bot aligned with the gold mineral
            System.out.println("ValleyXX Pos " + detector.getXPosition()); // Gold X pos in the view 270 - 370 is aligned
            System.out.println("ValleyXIsFound " + isFound); //is the gold in view
            System.out.println("ValleyXdetector IsFound " + detector.isFound()); //is the gold in view

            if (counter == 0)
            {
                foundString = "straight";
                foundRot = FoundRotationLocation.STRAIGHT;
            }

            if (isFound == false)
            {
                System.out.println("ValleyX cube is not found x=" + detector.getXPosition());
                counter = counter + 1;
                if (counter == 1)
                {
                    foundString = "right";
                    foundRot = FoundRotationLocation.RIGHT;
                    System.out.println("ValleyX turning right");
                    rotate(-10, 0.2); // turning right  to find gold
                    // turing off motors
                    motorRight.setPower(0);
                    motorLeft.setPower(0);
                }
                else if (counter == 2)
                {
                    foundString = "left";
                    foundRot = FoundRotationLocation.LEFT;
                    System.out.println("ValleyX turning left");
                    rotate(45, 0.2); // turning left to find gold
                    // turning off motors
                    motorRight.setPower(0);
                    motorLeft.setPower(0);
                }
                else if (counter == 3)
                {
                    System.out.println("ValleyX: Not found trying again");
                    rotate(-35, 0.2);
                    encoderDrive(0.3, 4, 4, 1);

                    counter = 0;
                }

            }
            else //is found
            {
                System.out.println("ValleyX cube is found " + foundString + " X=" + detector.getXPosition() + "Y=" + detector.getYPosition());
                if ((goldDetectorLeftX < detector.getXPosition()) && (goldDetectorRightX > detector.getXPosition())) {
                    System.out.println("ValleyX cube is aligned x=" + detector.getXPosition());
                    if (alignCount == 0) // drive forward to set up for second alignment
                    {
                        encoderDrive(0.6, 5, 5, 6);
                    } else //drive forward to knock off cube
                    {
                        //encoderDrive(1, 24, 24, 6);

                        //System.out.println("ValleyX cube found and knocked off");
                        break;
                    }
                    // pushed alignment parameters to the left to account for phone position on bot for second alignment
                    goldDetectorRightX -= 150;
                    goldDetectorLeftX -= 150;
                    goldDetectorRightX += 10;
                    goldDetectorLeftX -= 10;
                    alignCount++;  // increment to do second alignment
                }
                else //found but not aligned
                {
                    System.out.println("ValleyX cube found but not aligned x=" + detector.getXPosition());
                    if (detector.getXPosition() < goldDetectorLeftX) {
                        System.out.println("ValleyX turning right to align cube x=" + detector.getXPosition());
                        encoderDrive(0.2, 1, -1, 1);
                    } else {
                        System.out.println("ValleyX turning left to align cube x=" + detector.getXPosition());
                        encoderDrive(0.2, -1, 1, 1);
                    }
                }
            }
            System.out.println("ValleyX: counter " + counter);
        } // while op mode is active

        detector.disable();

        // initial steps for next part of autonomous after gold detection --> not working for this event

        System.out.println("ValleyX Gold Detector out of break");
        //encoderDrive(0.6, -25, -25, 6);

        if (foundRot == FoundRotationLocation.LEFT)
        {
            System.out.println("ValleyX found left");
            encoderDrive(1, 22, 22, 5);
            encoderDrive(0.6, -17, -17, 5);
            rotate(45, 0.2);
            encoderDrive(0.6, 35, 35, 6);
        }

        if (foundRot == FoundRotationLocation.STRAIGHT)
        {
            System.out.println("ValleyX found straight");
            encoderDrive(1, 20, 20, 5);
            encoderDrive(0.6, -17, -17, 5);
            rotate(70, 0.2);
            encoderDrive(0.6, 40, 40, 56);
        }

        if (foundRot == FoundRotationLocation.RIGHT)
        {
            System.out.println("ValleyX found right");
            encoderDrive(1, 24, 24, 5);
            encoderDrive(0.6, -17, -17, 5);
            rotate(95, 0.2);
            encoderDrive(0.6, 45, 45, 6);
        }


        System.out.println("ValleyX: ending");

    }
    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = -power;
            rightPower = power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = power;
            rightPower = -power;
        }
        else return;

        // set power to rotate.
        motorLeft.setPower(leftPower);
        motorRight.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {}

        // turn the motors off.
        motorRight.setPower(0);
        motorRight.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
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

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Ensure that the opmode is still active
        if (opModeIsActive())
        {

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
                    (motorLeft.isBusy() && motorRight.isBusy()))
            {

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

