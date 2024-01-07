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

package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcontroller.external.samples.RobotHardware;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

/*
 * This sample demonstrates how to run analysis during INIT
 * and then snapshot that value for later use when the START
 * command is issued. The pipeline is re-used from SkystoneDeterminationExample
 */
@Autonomous


public class BlueAllianceRight extends LinearOpMode {

    OpenCvWebcam webcam;
    PowerplayblueDeterminationExample.SkystoneDeterminationPipeline pipeline;
    PowerplayblueDeterminationExample.SkystoneDeterminationPipeline.SkystonePosition snapshotAnalysis = PowerplayblueDeterminationExample.SkystoneDeterminationPipeline.SkystonePosition.LEFT; // default
    private RobotHardware robot;
    public DcMotor lift;
    public CRServo intake1;
    public CRServo intake2;
    public CRServo gates;
    @Override
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
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });
        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        lift = hardwareMap.get(DcMotor.class, "lift");
        intake1 = hardwareMap.get(CRServo.class, "intake1");
        intake2 = hardwareMap.get(CRServo.class, "intake2");
        gates = hardwareMap.get(CRServo.class, "gates");
        intake2.setDirection(CRServo.Direction.REVERSE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        ((DcMotorEx) lift).setTargetPositionTolerance(5);

        /*
         * The START command just came in: snapshot the current analysis now
         * for later use. We must do this because the analysis will continue
         * to change as the camera view changes once the robot starts moving!
         */

        /*
         * Show that snapshot on the telemetry
         */
        telemetry.addData("Snapshot post-START analysis", snapshotAnalysis);
        telemetry.update();
Pose2d StartPose = new Pose2d(-32.4, 64.25, 179.1);
        drive.setPoseEstimate(StartPose);

        //left movements
        TrajectorySequence left = drive.trajectorySequenceBuilder(StartPose)


                .lineToConstantHeading(new Vector2d(-39.5, 30))



                .lineToConstantHeading(new Vector2d(-16,30))



                .lineToConstantHeading(new Vector2d(48, 43))


                .back(4)
                .build();




// right movements
        TrajectorySequence right = drive.trajectorySequenceBuilder(StartPose)
                .lineToSplineHeading(new Pose2d(-30, 34, Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(38, 30))
                .back(4)
                .build();


//middle movements
        TrajectorySequence middle = drive.trajectorySequenceBuilder(StartPose)
                .lineToSplineHeading(new Pose2d(-32.4, 36, 269))


                .turn(270)

        .lineToConstantHeading(new Vector2d(48, 36))


                .back(4)
                .build();





        switch (snapshotAnalysis) {
            case LEFT: {
               drive.followTrajectorySequence(left);

terminateOpModeNow();
            }
            case RIGHT: {
              drive.followTrajectorySequence(right);
            terminateOpModeNow();
            }
            case CENTER: {
              drive.followTrajectorySequence(middle);
              terminateOpModeNow();
            }
        }
    }
}

