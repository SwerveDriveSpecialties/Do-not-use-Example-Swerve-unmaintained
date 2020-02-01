/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.util;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.function.Supplier;

import com.swervedrivespecialties.exampleswerve.RobotMap;

/**
 * Add your docs here.
 */
public class util {
    public static double inchesToMeters(double inches){
        return inches / 39.37;
    }

    public static double metersToInches(double meters){
        return meters * 39.37;
	}
	
	public static Supplier<Boolean> getTrueSupplier(){
		return () -> true;
	}

	public static Supplier<Boolean> getFalseSupplier(){
		return () -> false;
	}

    public static DataLogger setupLogging(String mode) {
		DataLogger dataLogger;
				
		// see if the USB stick is plugged into to RoboRIO
		Path path = Paths.get(RobotMap.PRIMARY_LOG_FILE_PATH);
		Path alternatePath = Paths.get(RobotMap.ALTERNATE_LOG_FILE_PATH);
    	if (Files.exists(path)) {
    		try {
				dataLogger = new DataLogger(RobotMap.PRIMARY_LOG_FILE_PATH, mode);
					    		
	    		System.out.println("..Logging enabled to: " + dataLogger.getLogFilePathName());
			} catch (IOException e) {
				e.printStackTrace();
				
	    		dataLogger = null;
	    		
	    		System.out.println("..Error configuring Logging to: " + RobotMap.PRIMARY_LOG_FILE_PATH);
			}
    	}
    	else if (Files.exists(alternatePath)) {
    		try {
				dataLogger = new DataLogger(RobotMap.ALTERNATE_LOG_FILE_PATH, mode);
					    		
	    		System.out.println("..Logging enabled to: " + dataLogger.getLogFilePathName());
			} catch (IOException e) {
				e.printStackTrace();
				
	    		dataLogger = null;
	    		
	    		System.out.println("..Error configuring Logging to: " + RobotMap.ALTERNATE_LOG_FILE_PATH);
    		}
    	} else {
    		dataLogger = null;
    		
    		System.out.println("..Logging Disabled!");
    	}
    	return dataLogger;
	}

	public static double inchesToFeet(double inches){
		return inches / 12.;
	}

	public static double feetToInches(double feet){
		return feet * 12.;
	}
}
