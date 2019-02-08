package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

@Autonomous(name = "LEFT SIDE", group = "asdofijefj")
public class AutonomousWithSensorsLeft extends LinearOpMode
{
    final double TURN_TOLERANCE = 10;
    final double TURN_POWER = 0.15;
    HardwarePushbot robot = new HardwarePushbot();
    AutonomousState state;
    Orientation angles;

    @Override
    public void runOpMode() throws InterruptedException
    {
        autonomousStart();

        waitForStart();
        state = AutonomousState.TO_WALL;

        composeTelemetry();

        while (opModeIsActive())
        {
            /*
            Controls the current stage of the autonomous program based on an enum
             */
            switch (state)
            {
                case TO_WALL:
                    //autonomousToWall();
                    break;
                case TO_DEPOT:
                    autonomousToDepot();
                    break;
                case DROP_MARKER:
                    autonomousDropMarker();
                    break;
                case TO_CRATER:
                    autonomousToCrater();
                    break;
                case END:
                    break;
                default:
                    throw new InterruptedException("invalid state");
            }

            telemetry.update();
        }
    }

    private void composeTelemetry()
    {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(() ->
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles = robot.imu.getAngularOrientation(
                    AxesReference.INTRINSIC, AxesOrder.ZYX,
                    AngleUnit.DEGREES);
        });

        telemetry.addLine()
                .addData("status", () -> robot.imu.getSystemStatus().toShortString())
                .addData("calib", () -> robot.imu.getCalibrationStatus().toString());

        //heading is firstAngle
        telemetry.addLine()
                .addData("heading", () -> formatAngle(angles.angleUnit, angles.firstAngle))
                .addData("roll", () -> formatAngle(angles.angleUnit, angles.secondAngle))
                .addData("pitch", () -> formatAngle(angles.angleUnit, angles.thirdAngle));

        telemetry.addLine()
                .addData("dist", () -> String.format(Locale.US, "%.02f",
                        robot.distanceSensor.getDistance(DistanceUnit.CM)));
    }

    private void autonomousStart()
    {
        robot.init(hardwareMap);
    }

    private void autonomousToWall()
    {
        driveUntilDistance(10, DistanceUnit.CM, 1);
    }

    private void driveUntilDistance(double distance, DistanceUnit unit, double power)
    {
        power = Range.clip(power, -1, 1);
        robot.leftDrive.setPower(power);
        robot.rightDrive.setPower(power);

        while (robot.distanceSensor.getDistance(unit) > distance)
        {
            sleep(1000);
        }

        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

    }

    private void autonomousToDepot()
    {
        //turn towards depot
        rotate(90, 1);

        //go forward until hit wall
        driveUntilDistance(10, DistanceUnit.CM, 1);

    }

    private void autonomousDropMarker()
    {

    }

    private void autonomousToCrater()
    {

    }

    String formatAngle(AngleUnit angleUnit, double angle)
    {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees)
    {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    private void rotate(int degrees, double power)
    {
        power = Range.clip(power, 0, 1);
        robot.resetAngle();
        double leftPower, rightPower;

        //set the initial power of the motors depending on rotation
        if (degrees < 0)
        {
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {
            leftPower = -power;
            rightPower = power;
        }
        else
        {
            return;
        }

        robot.leftDrive.setPower(leftPower);
        robot.rightDrive.setPower(rightPower);

        //rotate within 3 degrees of error before stopping, and adjust as necessary
        while (Math.abs(robot.getAngle() - degrees) > TURN_TOLERANCE)
        {
            telemetry.addLine().addData("Angle: ", robot.getAngle());
            if (robot.getAngle() > degrees + TURN_TOLERANCE && leftPower < 0)
            {
                leftPower = power;
                rightPower = -power;
                robot.leftDrive.setPower(leftPower);
                robot.rightDrive.setPower(rightPower);
            }
            if (robot.getAngle() < degrees - TURN_TOLERANCE && leftPower > 0)
            {
                leftPower = -power;
                rightPower = power;
                robot.leftDrive.setPower(leftPower);
                robot.rightDrive.setPower(rightPower);
            }
            telemetry.update();
        }

        //stop the robot
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        sleep(1000);

        robot.resetAngle();
    }

    /*
        Represents the current "stage" of the autonomous program
        START: Robot has just been placed. Not moving.
        TO_WALL: Robot is driving directly perpendicular to wall towards wall
        TO_DEPOT: Robot is driving to depot
        DROP_MARKER: Robot is dropping marker in depot
        TO_CRATER: Robot is driving to crater to park
        END: Robot has parked. Not moving
     */
    public enum AutonomousState
    {
        START, TO_WALL, TO_DEPOT, DROP_MARKER, TO_CRATER, END;
    }
}
