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
    HardwarePushbot robot = new HardwarePushbot();
    AutonomousState state;
    Orientation angles;

    final double TURN_TOLERANCE = 10;
    final double TURN_POWER = 0.15;

    @Override
    public void runOpMode() throws InterruptedException
    {
        autonomousStart();

        waitForStart();
        state = AutonomousState.TO_WALL;

        while (opModeIsActive())
        {
            switch (state)
            {
                case TO_WALL:
                    rotate(91,TURN_POWER);
                    autonomousToWall();
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

            telemetry.addAction(() -> angles = robot.imu.getAngularOrientation(
                    AxesReference.INTRINSIC,
                    AxesOrder.ZYX,
                    AngleUnit.DEGREES));

            telemetry.addLine()
                    .addData("heading", () -> formatAngle(angles.angleUnit, angles.firstAngle))
                    .addData("roll", () -> formatAngle(angles.angleUnit, angles.secondAngle))
                    .addData("pitch", () -> formatAngle(angles.angleUnit, angles.thirdAngle));

            telemetry.addLine()
                    .addData("distance", () -> robot.distanceSensor.getDistance(DistanceUnit.CM));
        }
    }

    private void autonomousStart()
    {
        robot.init(hardwareMap);
    }

    private void autonomousToWall()
    {
        state = AutonomousState.TO_DEPOT;
    }

    private void autonomousToDepot()
    {

    }

    private void autonomousDropMarker()
    {
        rotate(180, 0.1);
        robot.flagDrop.setPosition(0.6);
        sleep(1000);
        robot.flagDrop.setPosition((0));
        state = AutonomousState.TO_CRATER;
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
