package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;

/*
 * This OpMode was written for the VuforiaDemo Basics video. This demonstrates basic principles of
 * using VuforiaDemo in FTC.
 */
@TeleOp(name = "( ͡° ͜ʖ ͡°) ")
public class VuforiaTest extends LinearOpMode
{
    //ULTRA SECRET KEY PLEASE DO NOT STEAL
    private static final String VUFORIA_KEY
            = "Aev5A3X/////AAABmYpr/G5Cm0omnR13eH0WhbFyr2MH6qsIOIq5bSAPiTX0oe1Y0NbEm98KXOe5uzEkF6nufhILTgETvg2JHD2iSBB6CPFJ7IMPyUQeYJWNcBnXqMXO4krRYmtmic5OXUKZ180TjdyWwqbCPVHrYtJRKcJ5lv5Tc9cphkkNnndomHoMDabic1iSSLUyt3U+GG2V1z1SXxSy8H7+Ba9lZoBY5vSEyTcM1rYypml6qndc3tfrBc75eNr5A6ukKaawV/pnUY7Ziz8D+HbDUvuSiXMfFvK9liFRhGcwmLLRehqF9HPRPqRtLMdxWn12FVyuvYjOSDkBDwgnTpvqNAxKsOCMU3BQECCkST+Oj3YcTeEdVdKd";
    // Variables to be used for later
    private VuforiaLocalizer vuforiaLocalizer;
    private VuforiaLocalizer.Parameters parameters;
    private VuforiaTrackables visionTargets;
    private VuforiaTrackable brainTarget;
    private VuforiaTrackable gwcTarget;
    private VuforiaTrackableDefaultListener listener;
    private OpenGLMatrix lastKnownLocation;
    private OpenGLMatrix phoneLocation;

//    private float robotX = 0;
//    private float robotY = 0;
//    private float robotAngle = 0;

    public void runOpMode() throws InterruptedException
    {
        setupVuforia();

        // We don't know where the robot is, so set it to the origin
        // If we don't include this, it would be null, which would cause errors later on
        lastKnownLocation = createMatrix(0, 0, 0, 0, 0, 0);

        waitForStart();

        // Start tracking the targets
        visionTargets.activate();

        while (opModeIsActive())
        {
            // Ask the listener for the latest information on where the robot is
            OpenGLMatrix latestLocation = listener.getUpdatedRobotLocation();

            // The listener will sometimes return null, so we check for that to prevent errors
            if (latestLocation != null)
            {
                lastKnownLocation = latestLocation;
            }

            float[] coordinates = lastKnownLocation.getTranslation().getData();

//            robotX = coordinates[0];
//            robotY = coordinates[1];
//            robotAngle = Orientation
//                    .getOrientation(lastKnownLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ,
//                            AngleUnit.DEGREES).thirdAngle;

            // Send information about whether the brainTarget is visible, and where the robot is
            telemetry.addData("Tracking " + brainTarget.getName(), listener.isVisible());
            telemetry.addData("Tracking " + gwcTarget.getName(), listener.isVisible());
            telemetry.addData("Last Known Location", formatMatrix(lastKnownLocation));

            // Send telemetry and idle to let hardware catch up
            telemetry.update();
            idle();
        }
    }

    private void setupVuforia()
    {
        // Setup parameters to create localizer
        parameters = new VuforiaLocalizer.Parameters(
                R.id.cameraMonitorViewId); // To remove the camera view from the screen, remove the R.id.cameraMonitorViewId
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.useExtendedTracking = false;
        vuforiaLocalizer = ClassFactory.getInstance().createVuforia(parameters);

        // These are the vision targets that we want to use
        // The string needs to be the name of the appropriate .xml file in the assets folder
        visionTargets = vuforiaLocalizer.loadTrackablesFromAsset("WomenWhoTech");
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 2);

        // Setup the brainTarget to be tracked
        brainTarget = visionTargets.get(1); // 1 corresponds to the glowy brain
        brainTarget.setName("Glowy Brain");
        brainTarget.setLocation(createMatrix(0, 500, 0, 90, 0, 90));

        //Setup the Girls Who Code target
        gwcTarget = visionTargets.get(0);
        gwcTarget.setName("Girls Who Code");
        gwcTarget.setLocation(createMatrix(0, 500, 0, 90, 0, 90));
        // Set phone location on robot
        phoneLocation = createMatrix(0, 225, 0, 90, 0, 0);

        // Setup listener and inform it of phone information
        listener = (VuforiaTrackableDefaultListener) brainTarget.getListener();
        listener.setPhoneInformation(phoneLocation, parameters.cameraDirection);
    }

    // Creates a matrix for determining the locations and orientations of objects
    // Units are millimeters for x, y, and z, and degrees for u, v, and w
    private OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w)
    {
        return OpenGLMatrix.translation(x, y, z).
                multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }

    // Formats a matrix into a readable string
    private String formatMatrix(OpenGLMatrix matrix)
    {
        return matrix.formatAsTransform();
    }
}

