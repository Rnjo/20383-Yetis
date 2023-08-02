package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//abhir
@TeleOp
public class Teleop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure yxour ID's match your configuration
        DcMotor leftFront = hardwareMap.dcMotor.get("leftFront");
        DcMotor leftRear = hardwareMap.dcMotor.get("leftRear");
        DcMotor rightFront = hardwareMap.dcMotor.get("rightFront");
        DcMotor rightRear = hardwareMap.dcMotor.get("rightRear");

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
//test
            leftFront.setPower(frontLeftPower);
            leftRear.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightRear.setPower(backRightPower);


            telemetry.addData("Flposition", leftFront.getCurrentPosition());
            telemetry.addData("Flisbusy", !leftFront.isBusy());
            telemetry.addData("FRposition", rightFront.getCurrentPosition());
            telemetry.addData("FRisbusy", !rightFront.isBusy());
            telemetry.addData("Blposition", leftRear.getCurrentPosition());
            telemetry.addData("Blisbusy", !leftRear.isBusy());
            telemetry.addData("Brposition", rightRear.getCurrentPosition());
            telemetry.addData("Brisbusy", !rightRear.isBusy());
        }
    }
}




































