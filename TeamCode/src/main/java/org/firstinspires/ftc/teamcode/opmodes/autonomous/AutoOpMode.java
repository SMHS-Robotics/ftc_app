package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.hardware.HardwareRobot;
import org.firstinspires.ftc.teamcode.utilities.PIDController;

public abstract class AutoOpMode extends LinearOpMode
{
    HardwareRobot robot;    //the robot in the opmode, duh
    Orientation angles;     //current rotation of the robot
    PIDController pidRotate;    //pid controller controlling rotation

    //rotates degrees at power
    public abstract void rotate(double degrees, double power);

    //sets up and displays telemetry on phone screen
    public abstract void composeTelemetry();
}
