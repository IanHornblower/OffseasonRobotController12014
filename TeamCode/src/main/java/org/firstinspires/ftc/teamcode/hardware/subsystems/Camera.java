package org.firstinspires.ftc.teamcode.hardware.subsystems;

import static org.firstinspires.ftc.teamcode.util.vision.AprilTagDetectionPipeline.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.teamcode.util.vision.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class Camera {

    HardwareMap hardwareMap;
    Telemetry telemetry;
    OpenCvCamera camera;
    String webcamName = "Webcam1";

    int resWidth = 1280;
    int resHeight = 720;

    public AprilTagDetectionPipeline pipeline;

    // Tags that are in use
    int LEFT = 9;
    int MIDDLE = 16;
    int RIGHT = 11;

    public enum State {
        LEFT(0),
        RIGHT(1),
        MIDDLE(2),
        NONE(0);

        int value;

        State(int value) {
            this.value = value;
        }

        public int getValue() {
            return  value;
        }



    }

    AprilTagDetection tagOfInterest = null;
    public State state = State.NONE;

    public Camera(HardwareMap hardwareMap, Telemetry telemetry) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        pipeline = new AprilTagDetectionPipeline(telemetry);
        camera.setPipeline(pipeline);
    }

    public OpenCvCamera getCamera() {
        return camera;
    }

    public void disableCamera() {
        camera.stopRecordingPipeline();
        camera.stopStreaming();
    }

    public State getSleeveLocation() {
        return state;
    }

    public void init() {
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
                camera.startStreaming(resWidth,resHeight, OpenCvCameraRotation.UPRIGHT);

            }

            @Override
            public void onError(int errorCode) {}
        });

        FtcDashboard.getInstance().startCameraStream(camera, 0);
        CameraStreamServer.getInstance().setSource(camera);
    }

    public void runInInit() {
        ArrayList<AprilTagDetection> currentDetections = pipeline.getLatestDetections();
        if(currentDetections.size() != 0) {
            boolean tagFound = false;

            for(AprilTagDetection tag : currentDetections) {
                if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                    tagOfInterest = tag;
                    tagFound = true;
                    break;
                }
            }

            if(tagFound) {
                //telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                //tagToTelemetry(tagOfInterest);
            }
            else {
                // telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null) {
                    //   telemetry.addLine("(The tag has never been seen)");
                }
                else {
                    //telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    //tagToTelemetry(tagOfInterest);
                }
            }

        }
        else {
            //telemetry.addLine("Don't see tag of interest :(");

            if(tagOfInterest == null) {
                //    telemetry.addLine("(The tag has never been seen)");
            }
            else {
                //   telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
            }

        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            // telemetry.addLine("Tag snapshot:\n");
            // tagToTelemetry(tagOfInterest);
            // telemetry.update();
        }
        else
        {
            // telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            // telemetry.update();
        }

        /* Actually do something useful */
        if(tagOfInterest == null){
            state = State.NONE;
        }else if(tagOfInterest.id == LEFT){
            state = State.LEFT;
        }else if(tagOfInterest.id == MIDDLE){
            state = State.MIDDLE;
        }else{
            state = State.RIGHT;
        }
    }

    public void shutdown() {
        disableCamera();
    }
}
