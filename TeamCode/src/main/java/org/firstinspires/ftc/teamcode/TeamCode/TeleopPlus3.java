package org.firstinspires.ftc.teamcode.TeamCode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp (name = "TeleopPlus3", group = "plus3robotics")
public class TeleopPlus3 extends LinearOpMode
{
    private DcMotor motorleft;
    private DcMotor motorright;
    private DcMotor motorlift;
    private DcMotor motorslide;
    private DcMotor motorREV;

    private Servo hook;
    private Servo slide;
    private Servo Lid;

    private TouchSensor armfailsafe;
//    private OpticalDistanceSensor forward;
//    private OpticalDistanceSensor backward;

    AnalogInput potentiometer;

    static final double SLIDE_OPEN = 0.1;  //Servo 4
    static final double SLIDE_CLOSE = 0.42; //Servo 4

    static final double HOOK_OPEN = .9;  //Hook Servo 3
    static final double HOOK_CLOSE = 1;  //Hook Servo 3

    static final double BUCKET_TOP_CLOSE = .48; // Servo 1
    static final double BUCKET_TOP_OPEN = 0.71;  //Servo 1

    static final double MOTOR_LIFT_0 = 0.582;
    static final double MOTOR_LIFT_20 = 0.735;
    static final double MOTOR_LIFT_90 = 1.332;
    static final double MOTOT_LIFT_100 = 1.523; //110 = 1.627

    @Override
    public void runOpMode()  throws InterruptedException{

        motorleft = hardwareMap.dcMotor.get("Lmotor");
        motorright = hardwareMap.dcMotor.get("Rmotor");
        motorlift = hardwareMap.dcMotor.get("Liftmotor");
        motorREV = hardwareMap.dcMotor.get("Intake");
        motorslide = hardwareMap.dcMotor.get("SlideExtension");

        hook = hardwareMap.servo.get("hangingclaw");
        slide = hardwareMap.servo.get("hangingservo");
        Lid = hardwareMap.servo.get("bucketTop");

        armfailsafe = hardwareMap.touchSensor.get("armfailsafe");
//        forward = hardwareMap.opticalDistanceSensor.get("frontsensor");
//        backward = hardwareMap.opticalDistanceSensor.get("backsensor");

        potentiometer = hardwareMap.analogInput.get("motorliftpot");
        motorleft.setDirection(DcMotor.Direction.REVERSE);

        double gamepadleftstickY;
        double gamepad1rightstickY;
        double gamepad2rightstickY;
        double gamepad2leftstickY;

        waitForStart();

        boolean lb_pressed = false;
        boolean rb_pressed = false;
        boolean down_pressed = false;
        boolean up_pressed = false;
        boolean direction_open = true;
        boolean lock_arm = false;
        double armVoltage = 0.0;

        // TODO: Set default state for servos here

        System.out.println("Plus3: voltage = " + potentiometer.getVoltage()); //0 degrees = 0.582, 20 degrees = 0.735, 90 degrees = 1.332, 110 degrees = 1.627
        while (opModeIsActive())
            {
                gamepadleftstickY = gamepad1.left_stick_y;
                gamepad1rightstickY = gamepad1.right_stick_y;
                gamepad2rightstickY = gamepad2.right_stick_y;
                gamepad2leftstickY = gamepad2.left_stick_y;

                telemetry.addData("gp1"," leftY=" +  gamepadleftstickY + "rightY=" + gamepad1rightstickY);

                telemetry.addData("armcontrol", "pot=" + potentiometer.getVoltage() + "touch=" + armfailsafe.getValue());

                telemetry.addData("gp2", " leftY2=" + gamepad2leftstickY + "rightY2+" + gamepad2rightstickY);

                telemetry.update();

                motorleft.setPower(gamepadleftstickY);
                motorright.setPower(gamepad1rightstickY);
                motorslide.setPower(gamepad2leftstickY);
                motorlift.setPower(-gamepad2rightstickY);

                //Values are negative to counteract the fact that the values are originally negative, this extra negative makes them positive
                if (gamepad2.right_stick_y != 0.0) {
                    lb_pressed = false;
                    down_pressed = false;
                    up_pressed = false;
                    rb_pressed = false;
                }
                //sets motor to 0 degrees
                if(gamepad2.dpad_left) {
                    Lid.setPosition(BUCKET_TOP_CLOSE);
                    lb_pressed = true;
                    rb_pressed = false;
                    down_pressed = false;
                    up_pressed = false;
                }
                if(lb_pressed && potentiometer.getVoltage() > MOTOR_LIFT_0)
                {
                    motorlift.setPower(-.1); //lowers the motor to 0 degrees
                }
                else if (lb_pressed){
                    lb_pressed = false;
                    motorlift.setPower(0);
                }
                //sets motor to 20 degrees
                if (gamepad2.dpad_down) {
                    Lid.setPosition(BUCKET_TOP_CLOSE);
                    down_pressed = true;
                    rb_pressed = false;
                    lb_pressed = false;
                    up_pressed = false;
                    direction_open = (potentiometer.getVoltage() < MOTOR_LIFT_20);
                    System.out.println("Plus3: Voltage"+potentiometer.getVoltage());
                }
                if (down_pressed && direction_open && potentiometer.getVoltage() < MOTOR_LIFT_20) {
                    motorlift.setPower(.7); //raises the motor to 90 degrees
                }
                else if (down_pressed && !direction_open && potentiometer.getVoltage() > MOTOR_LIFT_20) {
                    motorlift.setPower(-.1);
                }
//                else if (down_pressed) {
//                    direction_open = potentiometer.getVoltage() < MOTOR_LIFT_20;
//                }
                else if (down_pressed) {
                    down_pressed = false;
                    motorlift.setPower(0);
                }
                //sets motor to 90 degrees
                if (gamepad2.dpad_right) {
                    Lid.setPosition(BUCKET_TOP_CLOSE);
                    rb_pressed = true;
                    lb_pressed = false;
                    up_pressed = false;
                    down_pressed = false;
                    direction_open = (potentiometer.getVoltage() < MOTOR_LIFT_90);
                    System.out.println("Plus3: Voltage"+potentiometer.getVoltage());
                }
                if (rb_pressed && direction_open && potentiometer.getVoltage() < MOTOR_LIFT_90) {
                    motorlift.setPower(.7);
                }
                else if (rb_pressed && !direction_open && potentiometer.getVoltage() > MOTOR_LIFT_90) {
                    motorlift.setPower(-.1);
                }
                else if (rb_pressed) {
                    rb_pressed = false;
                    motorlift.setPower(0);
                }
                //sets motor to 100 degrees
                if (gamepad2.dpad_up) {
                    Lid.setPosition(BUCKET_TOP_CLOSE);
                    up_pressed = true;
                    down_pressed = false;
                    rb_pressed = false;
                    lb_pressed = false;
                    direction_open = (potentiometer.getVoltage() < MOTOT_LIFT_100);
                    System.out.println("Plus3: Voltage"+potentiometer.getVoltage());
                }
                if (up_pressed && direction_open && potentiometer.getVoltage() < MOTOT_LIFT_100) {
                    if (potentiometer.getVoltage() >= MOTOR_LIFT_90){
                        motorlift.setPower(.3);
                    }
                    else {
                        motorlift.setPower(.8); //raises the motor to 90 degrees
                    }
                }
                else if (up_pressed && !direction_open && potentiometer.getVoltage() > MOTOT_LIFT_100) {
                    motorlift.setPower(-.7);
                }
                else if (up_pressed) {
                    up_pressed = false;
                    motorlift.setPower(0);
                }
                if(gamepad2.right_bumper)
                {
                    motorleft.setPower(0);
                    motorright.setPower(0);
                    slide.setPosition(SLIDE_CLOSE);
                }
                if(gamepad2.left_bumper)
                {
                    motorleft.setPower(0);
                    motorright.setPower(0);
                    slide.setPosition(SLIDE_OPEN);
                }
                if(gamepad2.left_stick_button)
                {
                    motorleft.setPower(0);
                    motorright.setPower(0);
                    hook.setPosition(HOOK_OPEN);
                }
                if(gamepad2.right_stick_button)
                {
                    motorleft.setPower(0);
                    motorright.setPower(0);
                    hook.setPosition(HOOK_CLOSE);
                }
                if (gamepad2.x)
                {
                    System.out.println("Plus3: Lid Opening");
                    Lid.setPosition(BUCKET_TOP_OPEN);
                }
                else{
                    Lid.setPosition(BUCKET_TOP_CLOSE);
                }
                System.out.println("Plus3: left_trigger="+gamepad2.left_trigger);
                if (gamepad2.left_trigger >= 0.7 && !lock_arm){
                    lock_arm = true;
                    armVoltage = potentiometer.getVoltage();
                }
                else if (gamepad2.left_trigger < 0.7 && lock_arm){
                    lock_arm = false;
                    motorlift.setPower(0.0);
                }
                if (lock_arm){
                    if (potentiometer.getVoltage() > armVoltage + 0.1){
                        motorlift.setPower(-.5);
                    }
                    else if (potentiometer.getVoltage() < armVoltage - .1){
                        motorlift.setPower(.5);
                    }
                    else{
                        motorlift.setPower(0.0);
                    }
                }
                //motorREV.setPower(-1);
                if (gamepad1.right_bumper)
                {
                    System.out.println("Plus3: Spinning Forward");
                    motorREV.setPower(1);
                }
                else if (gamepad1.left_bumper)
                {
                    System.out.println("Plus3: Spinning backwards");
                    motorREV.setPower(-1);
                }
                else
                {
                    motorREV.setPower(0);
                }
                idle();
            }

    }

}
