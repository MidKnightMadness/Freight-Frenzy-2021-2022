package org.firstinspires.ftc.teamcode.Visual;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.MotionDetection;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

public class SampleVisual extends Visual{

    private VectorF position;
    private Orientation rotation;
    private VuforiaLocalizer vuforia;
    OpenGLMatrix lastLocation = new OpenGLMatrix();
    VuforiaTrackables targetsUltimateGoal;

    private TFObjectDetector tfod;
    STARTERSTACK starterstack;

    double ringOffset;

    // Constants for perimeter targets
    private static final float mmPerInch = 25.4f;
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField = 36 * mmPerInch;
    private static final float mmTargetHeight = 6 * mmPerInch;

    public void init(HardwareMap hardwareMap, Telemetry telemetry)
    {
        super.init(hardwareMap, telemetry);

        //  Instantiate the Vuforia engine
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = "Ae7oRjb/////AAABmV3pkVnpEU9Pv3XaN0o2EZ5ttngvTMliTd5nX0843lAXhah50oPXg63sdsiK9/BFMjXkw9lMippdx4bHQo5kycWr1GcFcv+QlVNEpSclUqu9Zzj4FYVl+J2ScSAXSyuCRWMRWd3AikCfhAtlwFe7dnMIfpVniU8Yr8o3YumS2/5LjNU2wIkiJak5IHlnugT414wsrzyqemO63BHn0Olbi3REkd61RxW3cE4lbSts3OI0GfnT57/Nw6/YfLAZQ69eCz0eEckVjPmbt7evb8lYo5gEpzm+wf5LVPaAzZWVj/gSQywzPKA8zoz4q6hl4zuAd3647Y3smuWVI8PpQzRwt5vP8d07Qt39p+/zEOrcGRDo";
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        targetsUltimateGoal = vuforia.loadTrackablesFromAsset("UltimateGoal");

        //set trackable positions
        //Red Alliance Target
        targetsUltimateGoal.get(2).setLocation(OpenGLMatrix
                .translation(0, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        //Blue Alliance Target
        targetsUltimateGoal.get(3).setLocation(OpenGLMatrix
                .translation(0, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        //Front Wall Target
        targetsUltimateGoal.get(4).setLocation(OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        //Blue Tower Goal Target
        targetsUltimateGoal.get(0).setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
        //Red Tower Goal Target
        targetsUltimateGoal.get(1).setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
        targetsUltimateGoal.activate();



        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters();
        tfodParameters.minResultConfidence = 0.5f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset("UltimateGoal.tflite", "Quad", "Single");
        tfod.activate();
    }

    public void update()
    {
        //check if any trackables are visible.
        for (VuforiaTrackable trackable : targetsUltimateGoal) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                //get the robot's position on the field (null if no information available)
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        //check if any rings visible
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null) {
            if(updatedRecognitions.size() == 0)  //no rings detected, starting stack A
                starterstack = STARTERSTACK.A;
            else{
                Recognition recognition = updatedRecognitions.get(0);
                String label = recognition.getLabel();
                if(label.equals("Single"))  //single ring detected, B
                    starterstack = STARTERSTACK.B;
                else if(label.equals("Quad"))  //4 rings detected, C
                    starterstack = STARTERSTACK.B;
                else
                    telemetry.addLine(label + " is not a known label");

                ringOffset = (recognition.getLeft() + recognition.getRight()) / 2;
            }
        }
    }

    public VectorF getPosition(){
        position = new VectorF(lastLocation.getTranslation().get(0), lastLocation.getTranslation().get(1), lastLocation.getTranslation().get(2));
        return position;
    }

    public Orientation getRotation(){
        return Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
    }

    public Visual.STARTERSTACK getStartStack()
    {
        return starterstack;
    }

    @Override
    public double getRingOffset() {
        return ringOffset;
    }
}
