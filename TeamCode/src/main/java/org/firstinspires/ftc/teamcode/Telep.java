package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.openftc.easyopencv.PipelineRecordingParameters;

//abhir
@TeleOp
public class Telep extends LinearOpMode {
    double arm_position;
    double arm_accel;
    double arm_max_position;
    double arm_min_position;
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor leftFront = hardwareMap.dcMotor.get("leftFront");
        DcMotor leftBack = hardwareMap.dcMotor.get("leftBack");
        DcMotor rightFront = hardwareMap.dcMotor.get("rightFront");
        DcMotor rightBack = hardwareMap.dcMotor.get("rightBack");
        DcMotor par0 = hardwareMap.dcMotor.get("par0");
        DcMotor par1 = hardwareMap.dcMotor.get("par1");
        DcMotor perp = hardwareMap.dcMotor.get("perp");
        DcMotor lift =hardwareMap.dcMotor.get("lift");
        Servo arm = hardwareMap.get(Servo.class, "arm");
        CRServo intake1 = hardwareMap.get(CRServo.class, "intake1");
        CRServo intake2 = hardwareMap.get(CRServo.class, "intake2");
        Servo gates = hardwareMap.get(Servo.class, "gates");



        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        intake2.setDirection(CRServo.Direction.REVERSE);
        waitForStart();

        if (isStopRequested()) return;
        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y * 0.5; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x * 0.5;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            leftFront.setPower(frontLeftPower);
            leftBack.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightBack.setPower(backRightPower);

//intake code
          if (gamepad2.a) {
                intake1.setPower(1);
                intake2.setPower(1);
            } else {
                intake1.setPower(0);
                intake2.setPower(0);
            }
//gate code
            if (gamepad2.b) {
                gates.setPosition(1);
            }else {
                gates.setPosition(0);
            }



//lift code
            lift.setPower(gamepad2.left_stick_y);




//for arm code borrow it from DriverModeFinalFlipback



              arm_position = arm.getPosition();
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
        arm.setPosition(arm_position);
    }


    telemetry.update();

            telemetry.addData("perp", perp.getCurrentPosition());
            telemetry.addData("par0", par0.getCurrentPosition());
            telemetry.addData("par1", par1.getCurrentPosition());
            telemetry.update();


        }
    }

