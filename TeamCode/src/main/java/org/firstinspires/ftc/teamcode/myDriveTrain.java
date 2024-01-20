package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;
//abhirAuton
public class myDriveTrain extends Bina {
    int iTics;                  // Variable to convert inches to iTics
    double dInchesToiTics;          // This is really a constant and will hold value of

                                // no of iTics required to move one inch
    public void toAndFro(double dInches) {
        dInchesToiTics = 47.17;
        iTics = (int) (dInches * dInchesToiTics);
        if (iTics>=0) {
            leftFront.setTargetPosition(abs(iTics));
            rightFront.setTargetPosition(abs(iTics));
            leftRear.setTargetPosition(abs(iTics));
            rightRear.setTargetPosition(abs(iTics));
            rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRear.setPower(0.7);
            rightFront.setPower(0.7);
            leftRear.setPower(0.7);
            leftFront.setPower(0.7);
            rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        else if (iTics<0)
        rightFront.setTargetPosition(abs(iTics));
        leftRear.setTargetPosition(abs(iTics));
        leftFront.setTargetPosition(abs(iTics));
        rightRear.setTargetPosition(abs(iTics));
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setPower(-0.7);
        rightFront.setPower(-0.7);
        leftRear.setPower(-0.7);
        leftFront.setPower(-0.7);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void leftAndRight(double dInches) {
        dInchesToiTics = 47.17;
        iTics = (int) (dInches * dInchesToiTics);
        if (iTics>=0) {
            leftFront.setTargetPosition(abs(iTics));
            rightFront.setTargetPosition(abs(iTics));
            leftRear.setTargetPosition(abs(iTics));
            rightRear.setTargetPosition(abs(iTics));
            rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRear.setPower(0.7);
            rightFront.setPower(-0.7);
            leftRear.setPower(-0.7);
            leftFront.setPower(0.7);
            rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        else if (iTics<0)
        leftRear.setTargetPosition(abs(iTics));
        rightFront.setTargetPosition(abs(iTics));
        leftFront.setTargetPosition(abs(iTics));
        rightRear.setTargetPosition(abs(iTics));
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setPower(-0.7);
        rightFront.setPower(0.7);
        leftRear.setPower(0.7);
        leftFront.setPower(-0.7);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void turn(boolean LR) {
        if (LR==true) {
            leftFront.setTargetPosition(800);
            rightFront.setTargetPosition(800);
            leftRear.setTargetPosition(800);
            rightRear.setTargetPosition(800);
            rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRear.setPower(-0.7);
            rightFront.setPower(-0.7);
            leftRear.setPower(0.7);
            leftFront.setPower(0.7);
            rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        }
        else if (LR==false) {
            leftFront.setTargetPosition(800);
            rightFront.setTargetPosition(800);
            leftRear.setTargetPosition(800);
            rightRear.setTargetPosition(800);
            rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRear.setPower(0.7);
            rightFront.setPower(0.7);
            leftRear.setPower(-0.7);
            leftFront.setPower(-0.7);
            rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        }
    }

}
