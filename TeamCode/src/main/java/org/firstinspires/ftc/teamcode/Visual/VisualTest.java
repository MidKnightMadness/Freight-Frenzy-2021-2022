package org.firstinspires.ftc.teamcode.Visual;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.MotionDetection;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
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

public class VisualTest extends Visual{

    private VectorF position;
    private Orientation rotation;
    private VuforiaLocalizer vuforia;
    OpenGLMatrix lastLocation = new OpenGLMatrix();
    VuforiaTrackables targetsUltimateGoal;

    private TFObjectDetector tfod;
    STARTERSTACK starterstack;

    public void init(HardwareMap hardwareMap, Telemetry telemetry)
    {
        super.init(hardwareMap, telemetry);

        //  Instantiate the Vuforia engine
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = "Ae7oRjb/////AAABmV3pkVnpEU9Pv3XaN0o2EZ5ttngvTMliTd5nX0843lAXhah50oPXg63sdsiK9/BFMjXkw9lMippdx4bHQo5kycWr1GcFcv+QlVNEpSclUqu9Zzj4FYVl+J2ScSAXSyuCRWMRWd3AikCfhAtlwFe7dnMIfpVniU8Yr8o3YumS2/5LjNU2wIkiJak5IHlnugT414wsrzyqemO63BHn0Olbi3REkd61RxW3cE4lbSts3OI0GfnT57/Nw6/YfLAZQ69eCz0eEckVjPmbt7evb8lYo5gEpzm+wf5LVPaAzZWVj/gSQywzPKA8zoz4q6hl4zuAd3647Y3smuWVI8PpQzRwt5vP8d07Qt39p+/zEOrcGRDo";
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        targetsUltimateGoal = vuforia.loadTrackablesFromAsset("UltimateGoal");
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
                String label = updatedRecognitions.get(0).getLabel();
                if(label.equals("Single"))  //single ring detected, B
                    starterstack = STARTERSTACK.B;
                else if(label.equals("Quad"))  //4 rings detected, C
                    starterstack = STARTERSTACK.B;
                else
                    telemetry.addLine(label + " is not a known label");
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

}
