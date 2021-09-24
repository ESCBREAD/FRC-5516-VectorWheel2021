// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.operation.OpManager;
import frc.operation.OpMode;
import frc.robot.operations.HeadlessDrive;
import frc.robot.operations.TestOp;
import frc.robot.operations.POVDrive;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.GyroSystem;

public class Robot extends TimedRobot {
    OpManager manager = new OpManager();

    // Subsystems
    static double wheels[][] = { { 1.0, 1.0 }, { -1.0, 1.0 }, { -1.0, -1.0 }, { 1.0, -1.0 } };
    // static int turnings[] = { 34, 36, 37, 35 };
    // static int drivings[] = { 30, 32, 33, 31 };
    // static int coders[] = { -1, -1, 9, -1 };
    //static double coderZeros[] = {0, 0, 3.83972, 0};

    // static int turnings[] = { 3, 7, 5, 1 };      //FL BL BR FR
    // static int drivings[] = { 4, 8, 6, 2 };
    // static int coders[] = { 10, 12, 11, 9 };
    //static int coders[] = { -1, -1, 11, -1 };
    static double coderZeros[] = {0, 0, 0, 0};

    static int turnings[] = { 3, 1, 7, 5 };      //FL FR BL BR
    static int drivings[] = { 4, 2, 8, 6 };
    static int coders[] = { 10, 9, 12, 11 };

    // static int turnings[] = { 3, 1, 5, 7 };         //FL FR BR BL
    // static int drivings[] = { 4, 2, 6, 8 };
    // static int coders[] = { 10, 9, 11, 12 };    


    public static Chassis chassis;

    public static GyroSystem gyro = new GyroSystem();

    // Joysticks
    public static Joystick stick = new Joystick(0);

    @Override
    public void robotInit() {
        // init chassis
        Translation2d wheelPos[] = new Translation2d[4];
        for (int i = 0; i < wheels.length; i++) {
            wheelPos[i] = new Translation2d(wheels[i][0], wheels[i][1]);

        }

        chassis = new Chassis(turnings, drivings, wheelPos, coders, coderZeros);

        // init manager
        manager.init();
        manager.register(chassis);
        manager.register(gyro);
    }

    @Override
    public void robotPeriodic() {
        manager.update();
    }

    @Override
    public void autonomousInit() {
        manager.setMode(OpMode.AUTONOMOUS);
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        manager.setMode(OpMode.TELEOP);
        //manager.startOperation(new TestOp(0.3));
        //manager.startOperation(new POVDrive(0.3));
        //manager.startOperation(new HeadlessDrive(0.3));
        manager.startOperation(new HeadlessDrive(0.6));
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void disabledInit() {
        manager.setMode(OpMode.DISABLED);

    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void testInit() {
        manager.setMode(OpMode.TEST);

    }

    @Override
    public void testPeriodic() {
    }
}
