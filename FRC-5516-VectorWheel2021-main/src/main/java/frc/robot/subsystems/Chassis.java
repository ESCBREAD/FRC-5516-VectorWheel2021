// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.operation.SubSystem;
import frc.robot.utils.VectorWheel;

public class Chassis extends SubSystem {
    VectorWheel wheels[];
    Translation2d wheelPositions[];

ShuffleboardTab shuffleTab = Shuffleboard.getTab("Chassis");
NetworkTableEntry ySpeedEntry = shuffleTab.add("Forward", 0).getEntry();
NetworkTableEntry xSpeedEntry = shuffleTab.add("Right", 0).getEntry();
NetworkTableEntry zSpeedEntry = shuffleTab.add("Turn", 0).getEntry();

NetworkTableEntry sSpeedEntry1 = shuffleTab.add("Speed1", 0).getEntry();
NetworkTableEntry sSpeedEntry2 = shuffleTab.add("Speed2", 0).getEntry();
NetworkTableEntry sSpeedEntry3 = shuffleTab.add("Speed3", 0).getEntry();
NetworkTableEntry sSpeedEntry4 = shuffleTab.add("Speed4", 0).getEntry();

NetworkTableEntry pWheelEntry1 = shuffleTab.add("WheelPos1", 0).getEntry();
NetworkTableEntry pWheelEntry2 = shuffleTab.add("WheelPos2", 0).getEntry();
NetworkTableEntry pWheelEntry3 = shuffleTab.add("WheelPos3", 0).getEntry();
NetworkTableEntry pWheelEntry4 = shuffleTab.add("WheelPos4", 0).getEntry();

NetworkTableEntry AngleEntry1 = shuffleTab.add("Angle1", 0).getEntry();
NetworkTableEntry AngleEntry2 = shuffleTab.add("Angle2", 0).getEntry();
NetworkTableEntry AngleEntry3 = shuffleTab.add("Angle3", 0).getEntry();
NetworkTableEntry AngleEntry4 = shuffleTab.add("Angle4", 0).getEntry();

NetworkTableEntry xWheelEntry1 = shuffleTab.add("xWheel1", 0).getEntry();
NetworkTableEntry xWheelEntry2 = shuffleTab.add("xWheel2", 0).getEntry();
NetworkTableEntry xWheelEntry3 = shuffleTab.add("xWheel3", 0).getEntry();
NetworkTableEntry xWheelEntry4 = shuffleTab.add("xWheel4", 0).getEntry();

NetworkTableEntry yWheelEntry1 = shuffleTab.add("yWheel1", 0).getEntry();
NetworkTableEntry yWheelEntry2 = shuffleTab.add("yWheel2", 0).getEntry();
NetworkTableEntry yWheelEntry3 = shuffleTab.add("yWheel3", 0).getEntry();
NetworkTableEntry yWheelEntry4 = shuffleTab.add("yWheel4", 0).getEntry();

// NetworkTableEntry gyroAngleEntry = shuffleTab.add("Angle", 0).withWidget(BuiltInWidgets.kGyro).getEntry();
// NetworkTableEntry gyroReadyEntry = shuffleTab.add("Ready", false).getEntry();

    /**
     * y+ is forward, x+ is right, turn is counter clockwise
     */
    public Chassis(int turnings[], int drivings[], Translation2d positions[], int coders[], double coderZeros[]) {
        super();

        // init wheels
        wheels = new VectorWheel[4];
        wheelPositions = positions.clone();

        for (int i = 0; i < 4; i++) {
            wheels[i] = new VectorWheel(turnings[i], drivings[i], coders[i], coderZeros[i]);
        }
    }

    /**
     * Test individual motor
     */
    public void testDrive(int n, double speed, double angle) {
        wheels[n].drive(speed, angle);
    }

    public VectorWheel getWheel(int n){
        return this.wheels[n];
    }

    /**
     * y+ is forward, x+ is right, turn is counter clockwise
     */
    public void drive(double forward, double right, double turnZ) {
        ySpeedEntry.setNumber(forward);
        xSpeedEntry.setNumber(right);
        zSpeedEntry.setNumber(turnZ);

        turnZ = turnZ * 0.5;
        Translation2d xyVec = new Translation2d(right, forward);

        // calc velocity of each wheel
        Translation2d[] wheelVelocities = new Translation2d[4];
        double maxLenth = 0;

        for (int i = 0; i < 4; i++) {
            Translation2d velocity = new Translation2d();
            velocity = velocity.plus(xyVec);

            Translation2d turnVec = new Translation2d();
            turnVec = turnVec.plus(wheelPositions[i]);
            turnVec = turnVec.rotateBy(new Rotation2d(2 * Math.PI / 4));
            turnVec = turnVec.times(turnZ);

            wheelVelocities[i] = velocity.plus(turnVec);

            double l = wheelVelocities[i].getNorm();
            if (l > maxLenth) {
                maxLenth = l;
            }
        }

        // scale down if greater than one
        if (maxLenth > 1) {
            for (int i = 0; i < 4; i++) {
                wheelVelocities[i] = wheelVelocities[i].times(1.0 / maxLenth);
            }
        }

        if (maxLenth < 0.1) {
            for (int i = 0; i < 4; i++) {
                wheels[i].drive(0, 0);
            }
        } else {
            // drive
            for (int i = 0; i < 4; i++) {
                Translation2d velocity = wheelVelocities[i];
// //for test                
double dSpeed = Math.sqrt(Math.pow(velocity.getX(), 2) + Math.pow(velocity.getY(), 2));
double dAngle = Math.atan2(velocity.getY(), velocity.getX());
if(i ==0 )
{
    sSpeedEntry1.setNumber(dSpeed);
    AngleEntry1.setNumber(dAngle);
}
else if (i ==1)
{
    sSpeedEntry2.setNumber(dSpeed);
    AngleEntry2.setNumber(dAngle);
}
else if (i ==2)
{
    sSpeedEntry3.setNumber(dSpeed);
    AngleEntry3.setNumber(dAngle);
}
else if (i ==3)
{
    sSpeedEntry4.setNumber(dSpeed);
    AngleEntry4.setNumber(dAngle);
}


                wheels[i].driveXY(velocity.getX(), velocity.getY());


int iPosition = wheels[i].getTunringPos();
if(i ==0 )
{
    pWheelEntry1.setNumber(iPosition);
    xWheelEntry1.setNumber(velocity.getX());
    yWheelEntry1.setNumber(velocity.getY());
}
else if (i ==1)
{
    pWheelEntry2.setNumber(iPosition);
    xWheelEntry2.setNumber(velocity.getX());
    yWheelEntry2.setNumber(velocity.getY());
}
else if (i ==2)
{
    pWheelEntry3.setNumber(iPosition);
    xWheelEntry3.setNumber(velocity.getX());
    yWheelEntry3.setNumber(velocity.getY());
}
else if (i ==3)
{
    pWheelEntry4.setNumber(iPosition);
    xWheelEntry4.setNumber(velocity.getX());
    yWheelEntry4.setNumber(velocity.getY());
}
            }
        }
    }

    @Override
    protected void update() {
        // send data to dashboards
    }
}
