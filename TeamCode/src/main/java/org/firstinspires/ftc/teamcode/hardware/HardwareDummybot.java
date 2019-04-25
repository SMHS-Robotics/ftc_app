package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
import org.firstinspires.ftc.teamcode.utilities.VuforiaPicture;

import java.util.ArrayList;
import java.util.List;

/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 * <p>
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 * <p>
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwareDummybot implements HardwareRobot
{

    //static final members
    public static final String TAG = "DUMMYBOT";
    //ULTRA SECRET KEY PLEASE DO NOT STEAL
    private static final String VUFORIA_KEY
            = "Aev5A3X/////AAABmYpr/G5Cm0omnR13eH0WhbFyr2MH6qsIOIq5bSAPiTX0oe1Y0NbEm98KXOe5uzEkF6nufhILTgETvg2JHD2iSBB6CPFJ7IMPyUQeYJWNcBnXqMXO4krRYmtmic5OXUKZ180TjdyWwqbCPVHrYtJRKcJ5lv5Tc9cphkkNnndomHoMDabic1iSSLUyt3U+GG2V1z1SXxSy8H7+Ba9lZoBY5vSEyTcM1rYypml6qndc3tfrBc75eNr5A6ukKaawV/pnUY7Ziz8D+HbDUvuSiXMfFvK9liFRhGcwmLLRehqF9HPRPqRtLMdxWn12FVyuvYjOSDkBDwgnTpvqNAxKsOCMU3BQECCkST+Oj3YcTeEdVdKd";
    /* Public OpMode members. */
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public BNO055IMU imu = null;
    /* local OpMode members. */
    HardwareMap hwMap = null;
    Orientation lastAngles = new Orientation();
    //Parameters for REV Expansion Hub
    double globalAngle = 0;
    // Variables to be used for later
    private VuforiaLocalizer vuforiaLocalizer;
    private VuforiaLocalizer.Parameters parameters;
    private VuforiaTrackables visionTargets;
    private List<VuforiaTrackable> trackableList;
    private List<VuforiaTrackableDefaultListener> listenerList;
    private OpenGLMatrix phoneLocation;
    private OpenGLMatrix lastKnownLocation;

    /* Constructor */
    public HardwareDummybot()
    {

    }

    private void setupVuforia(VuforiaPicture[] pics)
    {
        trackableList = new ArrayList<>();
        listenerList = new ArrayList<>();

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

        for (VuforiaPicture pic : pics)
        {
            //adds vuforia trackable objects from pics to trackableList
            VuforiaTrackable target = visionTargets.get(pic.ordinal());
            target.setName(pic.toString());
            target.setLocation(createMatrix(0, 500, 0, 90, 0, 90));

            trackableList.add(target); // adds
        }

        // Set phone location on robot
        phoneLocation = createMatrix(0, 225, 0, 90, 0, 0);

        // Setup listener and inform it of phone information
        for (int i = 0; i < pics.length; ++i)
        {
            //checks if an element in the trackable list is an instance of a default listener
            if (trackableList.get(i).getListener() instanceof VuforiaTrackableDefaultListener)
            {
                VuforiaTrackableDefaultListener listener =
                        (VuforiaTrackableDefaultListener) trackableList.get(i).getListener();
                // adds phone info to listener
                listener.setPhoneInformation(phoneLocation, parameters.cameraDirection);
                //adds listener to listener list
                listenerList.add(listener);
            }
            else
            {
                throw new ClassCastException("listener casted wrong");
            }

        }
    }

    public void init(HardwareMap hwMap)
    {
        //calls overloaded init function with empty array of pictures
        init(hwMap, new VuforiaPicture[] { });
    }

    //checks if listener corresponding to pic is visible
    public boolean isVisible(VuforiaPicture pic)
    {
        try
        {
            return getListener(pic).isVisible();
        }
        catch (IllegalArgumentException e)
        {
            return false;
        }
    }

    //gets listener that corresponds to pic
    public VuforiaTrackableDefaultListener getListener(VuforiaPicture pic)
    {
        for (int i = 0; i < trackableList.size(); ++i)
        {
            // checks if the trackable of pic is the same as trackable at index i
            if (visionTargets.get(pic.ordinal()).equals(trackableList.get(i)))
            {
                //returns matching trackable object
                return listenerList.get(i);
            }
        }
        throw new IllegalArgumentException("Picture not found in listener list");
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

    public OpenGLMatrix listenerLocation(VuforiaPicture pic)
    {
        try
        {
            lastKnownLocation = getListener(pic).getUpdatedRobotLocation();
        }
        catch (IllegalArgumentException e)
        {
        }
        return lastKnownLocation;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, VuforiaPicture... pics)
    {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDrive = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        // Set to REVERSE if using AndyMark motors
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        // Set to FORWARD if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile
                = "IMUTestCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Set Up Angle
        resetAngle();

        //AAAAAAAAAA
        setupVuforia(pics);
    }

    public double getAngle()
    {
        Orientation angles = imu
                .getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
        {
            deltaAngle += 360;
        }
        else if (deltaAngle > 180)
        {
            deltaAngle -= 360;
        }

        globalAngle += deltaAngle;
        lastAngles = angles;

        return globalAngle;
    }

    public void resetAngle()
    {
        lastAngles = imu
                .getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }
}