/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.Demo;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Abhir.myDriveTrain;
import org.firstinspires.ftc.teamcode.Vision.PowerplayRedDeterminationExample;
import org.firstinspires.ftc.teamcode.Vision.PowerplayblueDeterminationExample;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
@Autonomous


public class v4 extends myDriveTrain {
    OpenCvWebcam webcam;
    //OpenCvWebcam webcamRed;
    PowerplayblueDeterminationExample.SkystoneDeterminationPipeline pipeline;
    PowerplayblueDeterminationExample.SkystoneDeterminationPipeline.SkystonePosition snapshotAnalysis; /*= PowerplayblueDeterminationExample.SkystoneDeterminationPipeline.SkystonePosition.LEFT; // default*/
    PowerplayRedDeterminationExample.SkystoneDeterminationRedPipeline pipelineRed;
    //PowerplayRedDeterminationExample.SkystoneDeterminationRedPipeline.SkystonePosition snapshotAnalysisRed;/* = PowerplayRedDeterminationExample.SkystoneDeterminationRedPipeline.SkystonePosition.LEFT; // default*/

    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        /*
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCv,
         * you should take a look at {@link InternalCamera1Example} or its
         * webcam counterpart, {@link WebcamExample} first.
         */

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new PowerplayblueDeterminationExample.SkystoneDeterminationPipeline();
        pipelineRed = new PowerplayRedDeterminationExample.SkystoneDeterminationRedPipeline();
        //webcamRed = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(pipeline);
     // webcamRed.setPipeline(pipelineRed);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        /*webcamRed.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
               // webcamRed.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
*/
        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        FtcDashboard.getInstance().startCameraStream(webcam, 30);
        waitForStart();
        FtcDashboard.getInstance().startCameraStream(webcam, 30);
        telemetry.addData("Snapshot post-START analysis", snapshotAnalysis);
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Snapshot post-START analysis", snapshotAnalysis);
        telemetry.update();

        lift = hardwareMap.get(DcMotor.class, "lift");
        gates = hardwareMap.get(CRServo.class, "gates");
        intake1 = hardwareMap.get(CRServo.class, "intake1");
        intake2 = hardwareMap.get(CRServo.class, "intake2");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        arm1 = hardwareMap.get(Servo.class, "arm1");
        arm2 = hardwareMap.get(Servo.class, "arm2");
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ((DcMotorEx) rightRear).setTargetPositionTolerance(5);
        ((DcMotorEx) leftRear).setTargetPositionTolerance(5);
        ((DcMotorEx) rightFront).setTargetPositionTolerance(5);
        ((DcMotorEx) leftFront).setTargetPositionTolerance(5);
        ((DcMotorEx) lift).setTargetPositionTolerance(5);


        /*
         * The START command just came in: snapshot the current analysis now
         * for later use. We must do this because the analysis will continue
         * to change as the camera view changes once the robot starts moving!
         */
        snapshotAnalysis = pipeline.getAnalysis();
       // snapshotAnalysisRed = pipelineRed.getAnalysis();


        /*
         * Show that snapshot on the telemetry
         */

        //drive.setPoseEstimate(new Pose2d(-72, 12, 0));


        switch (snapshotAnalysis) {
            case LEFT: {
                if (isStopRequested()){
                    terminateOpModeNow();
                }

                FtcDashboard.getInstance().stopCameraStream();
                sleep(1000);
                leftAndRight(-24);
                sleep(1000);

                while (!isStopRequested()) {
                    Snapshotting(4);
                    Stopping(2);
                }

                break;
            }

            case CENTER: {
                if (isStopRequested()){
                    terminateOpModeNow();
                }

                FtcDashboard.getInstance().stopCameraStream();
                sleep(1000);
                toAndFro(24);
                sleep(1000);

                while (!isStopRequested()) {
                    Snapshotting(4);
                    Stopping(2);
                }

                break;

            }
            case RIGHT  : {
                if (isStopRequested()){
                    terminateOpModeNow();
                }

                FtcDashboard.getInstance().stopCameraStream();
                sleep(1000);
                leftAndRight(24);
                sleep(1000);

                while (!isStopRequested()) {
                    Snapshotting(4);
                   Stopping(2);
                }



                break;
            }

        }
    }

    void Snapshotting(int x) {
        for (int i=0; i<x; i++){
            FtcDashboard.getInstance().startCameraStream(webcam, 30);
            webcam.setPipeline(pipeline);
            webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            sleep(4500);
            PowerplayblueDeterminationExample.SkystoneDeterminationPipeline.SkystonePosition snapshotAnalysis2 = pipeline.getAnalysis();
            webcam.stopStreaming();
            switch (snapshotAnalysis2) {

                case LEFT: {
                    leftAndRight(-24);
                    break;
                }
                case CENTER: {
                    toAndFro(24);
                    break;
                }

                case RIGHT: {
                    leftAndRight(24);
                    break;
                }


            }

        }

    }


    void Stopping(double x) {
        for (int i=0; i<x; i++){

            FtcDashboard.getInstance().startCameraStream(webcam, 30);
            webcam.setPipeline(pipelineRed);
            webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            sleep(4500);
            PowerplayRedDeterminationExample.SkystoneDeterminationRedPipeline.SkystonePosition snapshotAnalysisK = pipelineRed.getAnalysis();
            webcam.stopStreaming();
            int garbage=4;

            switch (snapshotAnalysisK) {

                case LEFT:{
                  //  leftAndRight(-24);

                    terminateOpModeNow();
                    break;
                }
                case CENTER:{
                   // toAndFro(24);

                    terminateOpModeNow();
                    break;
                }

                case RIGHT: {
                   // leftAndRight(24);

                          terminateOpModeNow();
                    break;
                }
                case NONE: {
                   // toAndFro(-24);

                    garbage+=1;
                    break;
                }

            }

        }

    }
}


