package org.firstinspires.ftc.teamcode.Abhir;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class Bina extends LinearOpMode {

    public CRServo gates;

    private Servo launcher;
    public DcMotor leftRear;
    public DcMotor rightRear;
    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor lift;
    private DcMotor lift2;
    private TouchSensor reset;

    public Servo arm1;
    public Servo arm2;
    public CRServo intake1;
    public CRServo intake2;
    private DcMotor leftEncoder;
    private DcMotor rightEncoder;
    private DcMotor frontEncoder;


    double drive_slow_power;
    int drive_slow_velocity;
    double arm_position;
    int drive_max_velocity;
    int lift_pos;
    public int lift_max_position;
    double launcher_pos;
    boolean lift_reset_done;
    boolean is_reset_pressed;
    double lift_max_power_mult_down;
    int claw_closed_position;
    double gear_ratio;
    double arm_accel;
    double lift_max_power_mult_up;
    double claw_open_position;
    int lift_max_velocity;
    double lift_target_power;
    int lift_max_power;
    double lift_first_reset_power;
    int ticks_rev__fd_and_bk_;
    int lift_brake_threshold_down;
    public int lift_min_position;
    double circumference;
    int lift_brake_threshold_up;
    double arm_max_position;
    double arm_min_position;
    double arm_turn_Ok_position;
    int myDriveTrain_velocity;
    double drive_power;

    /**
     * Describe this function...
     Gamepad2 Left bumper - Slow tray outtake
     Gamepad2 Right bumper - Fast tray and rollers intake
     Gamepad2 A - One shot pixel delivery
     Gamepad2 B - Stop all intakes

     Gamepad1 Left bumper - Fast rollers outtake
     Gamepad1 Right bumper - Fast tray and rollers intake
     Gamepad1 A - Launch drone (keep pressed)
     Gamepad1 B - Stop all intakes
     */
    private void Claw_Control() {
        // ---------------------- Gate/intake/Launcher Code ----------------------
        lift_pos = lift.getCurrentPosition();

        if (gamepad2.left_bumper) {
            gates.setPower(-0.15);
        } else if (gamepad2.right_bumper) {
            gates.setPower(1);
            intake1.setPower(1);
            intake2.setPower(1);
        } else if (gamepad2.a && lift_pos >= lift_max_position*0.35) {
            arm1.setPosition(arm_min_position);
            arm2.setPosition(arm_min_position);
            sleep(1000);
            gates.setPower(-0.15);
        } else if (gamepad2.b) {
            intake1.setPower(0);
            intake2.setPower(0);
            gates.setPower(0);
        }

        if (gamepad1.left_bumper) {
            intake1.setPower(-1);
            intake2.setPower(-1);
        } else if (gamepad1.right_bumper) {
            gates.setPower(1);
            intake1.setPower(1);
            intake2.setPower(1);
        } else if (gamepad1.b) {
            intake1.setPower(0);
            intake2.setPower(0);
            gates.setPower(0);
        }

        // Drone launch
        if (gamepad1.a) {
            launcher.setPosition(0.5);
        } else {
            launcher.setPosition(0.18);
        }

        telemetry.update();
    }

    /**
     * Describe this function...
     */
    private void Timer(double Seconds) {
        ElapsedTime timer;

        timer = new ElapsedTime();
        timer.reset();
        while (timer.seconds() < Seconds) {
        }
    }

    /**
     * Describe this function...
     */
    private void Initialize_myDriveTrain() {
        // leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Describe this function...
     */
    private void Initialize_Lift_Arm() {
        arm1.setPosition(arm_max_position);
        arm2.setPosition(arm_max_position);
    }

    /**
     * Describe this function...
     */
    private void lift_telemetry() {

        telemetry.addData("lift target power", lift_target_power);
        telemetry.addData("lift pow", lift.getPower());
        telemetry.addData("lift speed mult up", lift_max_power_mult_up);
        telemetry.addData("lift speed mult down", lift_max_power_mult_down);
        telemetry.addData("lift pos", lift.getCurrentPosition());
        telemetry.addData("frontEncoder pos", frontEncoder.getCurrentPosition());
        telemetry.addData("lift vel", ((DcMotorEx) lift).getVelocity());
        telemetry.addData("gamepadX", gamepad2.right_stick_x);
        telemetry.addData("gamepadY", gamepad2.left_stick_y);
        telemetry.addData("arm1 pos", arm1.getPosition());
        telemetry.addData("arm2 pos", arm2.getPosition());
        telemetry.addData("arm accel", arm_accel);
        telemetry.addData("gates pos", gates.getPower());
        telemetry.addData("intake 1 pow", intake1.getPower());
        telemetry.addData("intake 2 pow", intake2.getPower());
        telemetry.addData("launcher pos", launcher.getPosition());
        telemetry.addData("lift targ pos tolerance",((DcMotorEx) lift).getTargetPositionTolerance());
        telemetry.addData("lift targ pos ", lift.getTargetPosition());
        telemetry.update();

    }

    /**
     * Describe this function...
     */
    private void Drive_Control() {
        double y;
        double x;
        double rx;
        double denominator;

        y  = -gamepad1.left_stick_y;
        x  = gamepad1.left_stick_x*1.1;
        rx = (gamepad1.right_stick_x * 0.9);

        if (gamepad1.right_stick_x != 0 || gamepad1.left_stick_y != 0 || gamepad1.left_stick_x != 0) {
            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            ((DcMotorEx) leftFront).setVelocity(drive_max_velocity * (((y + x) + rx) / denominator));
            ((DcMotorEx) leftRear).setVelocity(drive_max_velocity * ((y - x + rx) / denominator));
            ((DcMotorEx) rightFront).setVelocity(drive_max_velocity * (((y - x) - rx) / denominator));
            ((DcMotorEx) rightRear).setVelocity(drive_max_velocity * (((y + x) - rx) / denominator));
        } else if (gamepad1.right_stick_x == 0 && gamepad1.left_stick_y == 0 && gamepad1.left_stick_x == 0) {
            if (gamepad1.dpad_down || gamepad2.dpad_down) {
                ((DcMotorEx) leftRear).setVelocity(-drive_slow_velocity);
                ((DcMotorEx) rightRear).setVelocity(-drive_slow_velocity);
                ((DcMotorEx) leftFront).setVelocity(-drive_slow_velocity);
                ((DcMotorEx) rightFront).setVelocity(-drive_slow_velocity);

            } else if (gamepad1.dpad_up || gamepad2.dpad_up) {
                ((DcMotorEx) leftRear).setVelocity(drive_slow_velocity);
                ((DcMotorEx) rightRear).setVelocity(drive_slow_velocity);
                ((DcMotorEx) leftFront).setVelocity(drive_slow_velocity);
                ((DcMotorEx) rightFront).setVelocity(drive_slow_velocity);
            } else if (gamepad1.dpad_left || gamepad2.dpad_left) {
                ((DcMotorEx) leftRear).setVelocity(drive_slow_velocity);
                ((DcMotorEx) rightRear).setVelocity(-drive_slow_velocity);
                ((DcMotorEx) leftFront).setVelocity(-drive_slow_velocity);
                ((DcMotorEx) rightFront).setVelocity(drive_slow_velocity);
            } else if (gamepad1.dpad_right || gamepad2.dpad_right) {
                ((DcMotorEx) leftRear).setVelocity(-drive_slow_velocity);
                ((DcMotorEx) rightRear).setVelocity(drive_slow_velocity);
                ((DcMotorEx) leftFront).setVelocity(drive_slow_velocity);
                ((DcMotorEx) rightFront).setVelocity(-drive_slow_velocity);
            } else {
                ((DcMotorEx) leftRear).setVelocity(0);
                ((DcMotorEx) rightRear).setVelocity(0);
                ((DcMotorEx) leftFront).setVelocity(0);
                ((DcMotorEx) rightFront).setVelocity(0);
            }
        }
        // Denominator is the largest motor power
        // (absolute value) or 1.
        // This ensures all the powers maintain
        // the same ratio, but only when at least one is
        // out of the range [-1, 1].
        // Make sure your ID's match your configuration
    }

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        int slow_velocity;
        double lift_power_incr;
        int lift_high_junction_max;

        gates = hardwareMap.get(CRServo.class, "gates");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        lift = hardwareMap.get(DcMotor.class, "lift");
        lift2 = hardwareMap.get(DcMotor.class, "frontEncoder");
        frontEncoder = hardwareMap.get(DcMotor.class, "frontEncoder");
        arm1 = hardwareMap.get(Servo.class, "arm1");
        arm2 = hardwareMap.get(Servo.class, "arm2");
        launcher = hardwareMap.get(Servo.class, "launcher");
        leftEncoder = hardwareMap.dcMotor.get("leftEncoder");
        rightEncoder = hardwareMap.dcMotor.get("rightEncoder");
        frontEncoder = hardwareMap.dcMotor.get("frontEncoder");
        intake1 = hardwareMap.get(CRServo.class, "intake1");
        intake2 = hardwareMap.get(CRServo.class, "intake2");
        reset = hardwareMap.get(TouchSensor.class, "reset");
        arm2.setDirection(Servo.Direction.REVERSE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        intake2.setDirection(CRServo.Direction.REVERSE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ((DcMotorEx) lift).setTargetPositionTolerance(5);
        frontEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setDirection(Servo.Direction.REVERSE);

        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //frontEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive_max_velocity = 1700;
        lift_reset_done = false;
        is_reset_pressed= reset.isPressed();
        lift_max_power = 1;
        lift_min_position = 0;
        lift_max_position = 3150;
        lift_max_power_mult_up = 1;
        lift_max_power_mult_down = 0.7;
        drive_slow_velocity=600;
        lift_power_incr = 0.1;
        lift_max_velocity = 0;
        arm_max_position = 0.48;
        arm_min_position = 0.0;
        arm_position = 0.5;
        arm_accel = 0;
        arm_turn_Ok_position = 0.41;
        claw_open_position = 0.55;
        claw_closed_position = 0;
        lift_high_junction_max = 1;
        lift_max_velocity = 2100;
        drive_power = 0.7;
        drive_slow_power = 0.2;


        // Do some initialization
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Initialize_Lift_Arm();
        Initialize_myDriveTrain();
        lift_telemetry();
        telemetry.update();

        // Wait for Start button
        waitForStart();
        telemetry.update();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                Drive_Control();
                Lift_Control();
                Arm_Control();
                Claw_Control();
                lift_telemetry();
                //drive_telemetry();
                // Return_home();
                //Deliver_cone();
                //position_zero();
                //position_max();
                telemetry.update();
            }
        }
    }
    public void position_zero() {
        if ( gamepad2.x) {
            gates.setPower(0);
            arm1.setPosition(arm_max_position);
            arm2.setPosition(arm_max_position);
            sleep(500);
            lift.setTargetPosition(lift_min_position);
            //frontEncoder.setTargetPosition(lift_min_position);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //frontEncoder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(lift_max_power);
            lift2.setPower(lift_max_power);
            //frontEncoder.setPower(lift_max_power);
            while(lift.isBusy() || lift2.isBusy()){
                telemetry.update();
            }
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //frontEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

    }
    /**
     * Describe this function...
     */
    private void drive_telemetry() {
        telemetry.addData("leftRear pow", leftRear.getPower());
        telemetry.addData("leftRear vel", ((DcMotorEx) leftRear).getVelocity());
        telemetry.addData("leftFront pow", leftFront.getPower());
        telemetry.addData("leftFront vel", ((DcMotorEx) leftFront).getVelocity());
        telemetry.addData("rightRear pow", rightRear.getPower());
        telemetry.addData("rightRear vel", ((DcMotorEx) rightRear).getVelocity());
        telemetry.addData("rightFront pow", rightFront.getPower());
        telemetry.addData("rightFront vel", ((DcMotorEx) rightFront).getVelocity());
        telemetry.addData("leftEncoder pos", leftEncoder.getCurrentPosition());
        telemetry.addData("rightEncoder pos", rightEncoder.getCurrentPosition());
        telemetry.addData("frontEncoder pos", frontEncoder.getCurrentPosition());
        telemetry.addData("left Back pos", leftRear.getCurrentPosition());
        telemetry.addData("Left Front pos", leftFront.getCurrentPosition());
        telemetry.addData("Right Back pos", rightRear.getCurrentPosition());
        telemetry.addData("Right Front pos", rightFront.getCurrentPosition());}

    /**
     * Describe this function...
     */
    private void Arm_Control() {
        // ---------------------- Arm Code ----------------------
        arm_position = arm1.getPosition();
        if (gamepad2.right_stick_y < 0) {
            arm_accel = -gamepad2.right_stick_y * 0.06;
        } else if (gamepad2.right_stick_y > 0) {
            arm_accel = -gamepad2.right_stick_y * 0.03;
        } else {
            arm_accel = 0;
        }
        arm_position = arm_position - arm_accel;
        if (arm_position > arm_max_position) {
            arm_position = arm_max_position;
        } else {
            if (arm_position <= arm_min_position) {
                arm_position = arm_min_position;
            }
        }
        arm1.setPosition(arm_position);
        arm2.setPosition(arm_position);
    }

    /**
     * Describe this function...
     */
    private void Lift_Control() {
        double lift_power;

        lift_pos = lift.getCurrentPosition();

        // ---------------------- Lift Code ----------------------
        lift_max_power_mult_up = 1;
        lift_max_power_mult_down = 1;
        if (reset.isPressed() && !lift_reset_done) {
            gates.setPower(0);
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift_pos = lift.getCurrentPosition();
            arm1.setPosition(arm_max_position);
            arm2.setPosition(arm_max_position);
            lift_reset_done = true;
        }
        lift_pos = lift.getCurrentPosition();
        lift_power = lift.getPower();

        if (!reset.isPressed() && lift_reset_done) {
            lift_reset_done = false;
        }

        if (gamepad2.left_stick_y < 0 && lift_pos <= lift_max_position) {
            lift_target_power = -(lift_max_velocity*lift_max_power_mult_up * lift_max_power * gamepad2.left_stick_y);
        } else if (gamepad2.left_stick_y > 0 && lift_pos >= lift_max_position*0.35 && !reset.isPressed()) {
            arm1.setPosition(arm_max_position*0.90);
            arm2.setPosition(arm_max_position*0.90);
            gates.setPower(0);
            lift_target_power = -(lift_max_velocity*lift_max_power_mult_down * lift_max_power * gamepad2.left_stick_y);
        } else if (gamepad2.left_stick_y > 0 && !reset.isPressed()) {
            lift_target_power = -(lift_max_velocity*lift_max_power_mult_down * lift_max_power * gamepad2.left_stick_y);
        } else {
            lift_target_power = 0;
        }

        lift_power = lift_target_power;
        ((DcMotorEx) lift).setVelocity(lift_power);
        ((DcMotorEx) lift2).setVelocity(lift_power);
    }
}