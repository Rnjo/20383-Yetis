package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "DriverModeFinalFlipbackAutomatedSureshbeltdp (Blocks to Java)")
public class DriverModeFinalFlipbackAutomatedSureshbeltdp extends LinearOpMode {

    private Servo gates;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor swivel;
    private DcMotor lift;     private DcMotor lift2;

    private Servo arm1;    private Servo arm2;

    private TouchSensor reset;

    double drive_slow_power;
    double arm_position;
    int drive_max_velocity;
    int lift_pos;
    int lift_max_position;
    boolean lift_reset_done;
    double lift_max_power_mult_down;
    int claw_closed_position;
    double gear_ratio;
    double arm_accel;
    double lift_max_power_mult_up;
    double claw_open_position;
    int lift_max_velocity;
    double lift_target_power;
    int lift_max_power;
    int ticks_rev__fd_and_bk_;
    int lift_brake_threshold_down;
    int lift_min_position;
    double circumference;
    int lift_brake_threshold_up;
    double arm_max_position;
    int arm_min_position;
    double arm_turn_Ok_position;
    int Drivetrain_velocity;
    double drive_power;

    /**
     * Describe this function...
     */
    private void Claw_Control() {
        // ---------------------- Claw Code ----------------------

        if (gamepad2.a) {
            gates.setPosition(1);


        } else if (gamepad2.b) {
            gates.setPosition(0);
        }
        lift_telemetry();
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
private void position_zero() {
        if (gamepad1.a) {
            arm1.setPosition(0);
            arm2.setPosition(0);
            sleep(2000);
            lift.setTargetPosition(0);
            lift2.setTargetPosition(0);
lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            gates.setPosition(0);

        }
    }

    /**
     * Describe this function...
     */
    private void Initialize_DriveTrain() {
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Describe this function...
     */


    /**
     * Describe this function...
     */
    private void lift_telemetry() {
        telemetry.addData("lift reset", reset.isPressed());
        telemetry.addData("lift brake", lift.getZeroPowerBehavior());
        telemetry.addData("lift reset done", lift_reset_done);
        telemetry.addData("lift target power", lift_target_power);
        telemetry.addData("lift pow", lift.getPower());
        telemetry.addData("lift speed mult up", lift_max_power_mult_up);
        telemetry.addData("lift speed mult down", lift_max_power_mult_down);
        telemetry.addData("lift pos", lift_pos);
        telemetry.addData("lift vel", ((DcMotorEx) lift).getVelocity());
        telemetry.addData("gamepadX", gamepad2.right_stick_x);
        telemetry.addData("gamepadY", gamepad2.left_stick_y);
        telemetry.addData("arm pos", arm1.getPosition());
        telemetry.addData("arm pos", arm2.getPosition());

        telemetry.addData("arm accel", arm_accel);
        telemetry.addData("gates pos", gates.getPosition());
    }

    /**
     * Describe this function...
     */


    /**
     * Describe this function...
     */
    private void Deliver_cone() {
        if (gamepad2.y) {
            if (lift_pos >= 60) {
                lift_reset_done = false;
                lift_max_power_mult_down = 1;
            }
            gates.setPosition(0);
            Timer(0.6);
            arm1.setPosition(1);
            arm2.setPosition(1);

            lift.setTargetPosition(1200);
            lift2.setTargetPosition(1200);

            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            lift.setPower(lift_max_power * 1);
            Timer(2);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Describe this function...
     */
    private void Drive_Control() {
        float y;
        float x;
        double rx;
        double denominator;

        y = -gamepad1.left_stick_y;
        x = -gamepad1.left_stick_x;
        rx = -(gamepad1.right_stick_x * -0.6);
        if (gamepad1.right_stick_x != 0 || gamepad1.left_stick_y != 0 || gamepad1.left_stick_x != 0) {
            denominator = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(y), Math.abs(x), Math.abs(rx))), 1));
            leftFront.setPower(drive_power * (((y - x) + rx) / denominator));
            leftBack.setPower(drive_power * ((y + x + rx) / denominator));
            rightFront.setPower(drive_power * (((y + x) - rx) / denominator));
            rightBack.setPower(drive_power * (((y - x) - rx) / denominator));
        } else if (gamepad1.right_stick_x == 0 && gamepad1.left_stick_y == 0 && gamepad1.left_stick_x == 0) {
            if (gamepad1.dpad_up || gamepad2.dpad_up) {
                leftBack.setPower(drive_slow_power);
                rightBack.setPower(drive_slow_power);
                leftFront.setPower(drive_slow_power);
                rightFront.setPower(drive_slow_power);
            } else if (gamepad1.dpad_down || gamepad2.dpad_down) {
                leftBack.setPower(-drive_slow_power);
                rightBack.setPower(-drive_slow_power);
                leftFront.setPower(-drive_slow_power);
                rightFront.setPower(-drive_slow_power);
            } else if (gamepad1.dpad_left || gamepad2.dpad_left) {
                leftBack.setPower(drive_slow_power);
                rightBack.setPower(-drive_slow_power);
                leftFront.setPower(-drive_slow_power);
                rightFront.setPower(drive_slow_power);
            } else if (gamepad1.dpad_right || gamepad2.dpad_right) {
                leftBack.setPower(-drive_slow_power);
                rightBack.setPower(drive_slow_power);
                leftFront.setPower(drive_slow_power);
                rightFront.setPower(-drive_slow_power);
            } else {
                leftBack.setPower(0);
                rightBack.setPower(0);
                leftFront.setPower(0);
                rightFront.setPower(0);
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

        gates = hardwareMap.get(Servo.class, "gates");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        swivel = hardwareMap.get(DcMotor.class, "swivel");
        lift = hardwareMap.get(DcMotor.class, "lift");
        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        arm1 = hardwareMap.get(Servo.class, "arm1");
        arm2 = hardwareMap.get(Servo.class, "arm2");

        reset = hardwareMap.get(TouchSensor.class, "reset");
arm2.setDirection(Servo.Direction.REVERSE);
lift2.setDirection(DcMotorSimple.Direction.REVERSE);
        drive_max_velocity = 1250;
        slow_velocity = 500;
        lift_reset_done = false;
        lift_max_power = 1;
        lift_min_position = 0;
        lift_max_position = 1200;
        lift_max_power_mult_up = 1;
        lift_max_power_mult_down = 1;
        lift_power_incr = 0.1;
        lift_max_velocity = 0;
        arm_max_position = 0.8;
        arm_min_position = 0;
        arm_position = 0.5;
        arm_accel = 0;
        arm_turn_Ok_position = 0.41;
        claw_open_position = 0.55;
        claw_closed_position = 0;
        lift_high_junction_max = 1;
        Drivetrain_velocity = 1400;
        circumference = 9.3886;
        ticks_rev__fd_and_bk_ = 560;
        gear_ratio = 0.56;
        lift_max_velocity = 2100;
        drive_power = 0.7;
        drive_slow_power = 0.2;
        Initialize_DriveTrain();
       // Initialize_Lift();
        lift_telemetry();
        telemetry.update();
        // Wait for Start button
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                Drive_Control();
                Lift_Control();
                Arm_Control();
                Claw_Control();
                lift_telemetry();
               // Return_home();
                Deliver_cone();
                telemetry.update();
            }
        }
    }

    /**
     * Describe this function...
     */
    private void drive_telemetry() {
        telemetry.addData("leftback pow", leftBack.getPower());
        telemetry.addData("leftback vel", ((DcMotorEx) leftBack).getVelocity());
        telemetry.addData("leftFront pow", leftFront.getPower());
        telemetry.addData("leftFront vel", ((DcMotorEx) leftFront).getVelocity());
        telemetry.addData("rightback pow", rightBack.getPower());
        telemetry.addData("rightback vel", ((DcMotorEx) rightBack).getVelocity());
        telemetry.addData("rightFront pow", rightFront.getPower());
        telemetry.addData("rightFront vel", ((DcMotorEx) rightFront).getVelocity());
    }

    /**
     * Describe this function...
     */


    /**
     * Describe this function...
     */




    /**
     * Describe this function...
     */
    private void stop_and_reset() {
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void drive(double fr_distance, double fl_distance, double bl_distance, double br_distance) {
        stop_and_reset();
        rightFront.setTargetPosition((int) ((fr_distance / circumference) * ticks_rev__fd_and_bk_ * gear_ratio));
        leftFront.setTargetPosition((int) ((fl_distance / circumference) * ticks_rev__fd_and_bk_ * gear_ratio));
        leftBack.setTargetPosition((int) ((bl_distance / circumference) * ticks_rev__fd_and_bk_ * gear_ratio));
        rightBack.setTargetPosition((int) ((br_distance / circumference) * ticks_rev__fd_and_bk_ * gear_ratio));
        ((DcMotorEx) leftFront).setTargetPositionTolerance(5);
        ((DcMotorEx) rightFront).setTargetPositionTolerance(5);
        ((DcMotorEx) leftBack).setTargetPositionTolerance(5);
        ((DcMotorEx) rightBack).setTargetPositionTolerance(5);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) rightFront).setVelocity(Drivetrain_velocity);
        ((DcMotorEx) leftFront).setVelocity(Drivetrain_velocity);
        ((DcMotorEx) rightBack).setVelocity(Drivetrain_velocity);
        ((DcMotorEx) leftBack).setVelocity(Drivetrain_velocity);
        Timer(0.5);
        stop_and_reset();
    }

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
        arm_position = arm_position + arm_accel;
        if (arm_position > arm_max_position) {
            arm_position = arm_max_position;
        } else {
            if (arm_position <= arm_min_position) {
                arm_position = arm_min_position;
            }
        }
        arm1.setPosition(arm_position);
    }

    /**
     * Describe this function...
     */
    private void Lift_Control() {
        double lift_power;

        // ---------------------- Lift Code ----------------------
        lift_pos = lift.getCurrentPosition();
        lift_max_power_mult_up = 1;
        lift_max_power_mult_down = 1;
        if (reset.isPressed()) {
            if (!lift_reset_done) {
                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                lift_pos = lift.getCurrentPosition();
                lift_reset_done = true;
            }
            lift_max_power_mult_down = 0;
        }
        if (lift_pos >= 60) {
            lift_reset_done = false;
        }
        lift_pos = lift.getCurrentPosition();
        lift_power = lift.getPower();
        if (gamepad2.left_stick_y < 0 && lift_pos <= lift_max_position) {
            lift_target_power = -(lift_max_power_mult_up * lift_max_power * gamepad2.left_stick_y);
        } else if (gamepad2.left_stick_y > 0) {
            lift_target_power = -(lift_max_power_mult_down * lift_max_power * gamepad2.left_stick_y);
        } else {
            if (lift.getCurrentPosition() > 200) {
                lift_target_power = 0.001;
            } else {
                lift_target_power = 0;
            }
        }
        lift_power = lift_target_power;
        lift.setPower(lift_power);
    }
}