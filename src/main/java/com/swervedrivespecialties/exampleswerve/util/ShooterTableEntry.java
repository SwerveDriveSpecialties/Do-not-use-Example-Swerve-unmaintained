/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.util;

/**
 * Add your docs here.
 */
public class ShooterTableEntry {
    public final int Index;
	public final double DistanceInFeet;
	public final int MotorTargetRPM;
	public final double ActuatorVal;
	


	//============================================================================================
	// constructors follow
	//============================================================================================
	public ShooterTableEntry(int index, double distanceInFeet, int MotorRPM, double ActuatorValue) {
		Index = index;
		DistanceInFeet = distanceInFeet;
		MotorTargetRPM = MotorRPM;
		ActuatorVal = ActuatorValue;
		
   
    }



	
}
