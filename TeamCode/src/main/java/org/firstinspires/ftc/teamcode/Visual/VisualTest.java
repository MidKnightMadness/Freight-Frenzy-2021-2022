package org.firstinspires.ftc.teamcode.Visual;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.MotionDetection;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

public class VisualTest extends Visual{

    private VectorF position;
    private Orientation rotation;
    private VuforiaLocalizer vuforia;
    OpenGLMatrix lastLocation = new OpenGLMatrix();
    VuforiaTrackables targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");

    public void init(HardwareMap hardwareMap, Telemetry telemetry)
    {
        super.init(hardwareMap, telemetry);
    }

    public void update()
    {
        boolean targetVisible;
        //check if any trackables are visible.
        targetVisible = false;
        for (VuforiaTrackable trackable : targetsUltimateGoal) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                targetVisible = true;
                //get the robot's position on the field (null if no information available)
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }
    }

    public VectorF getPosition(){
        position = new VectorF(lastLocation.getTranslation().get(0), lastLocation.getTranslation().get(1), lastLocation.getTranslation().get(2));
        return position;
    }

    /*
    public Orientation getRotation(){
        rotation = new Orientation(Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES).firstAngle, Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES).secondAngle, Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES).thirdAngle);
    }
    */

    public Visual.STARTERSTACK getStartStack()
    {
        return null;
    }

}
