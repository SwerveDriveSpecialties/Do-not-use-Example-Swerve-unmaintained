/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.util;

import java.util.ArrayList;
import java.util.List;

// This is a "data entity" class that hold the data to logged.
// Subsystem classes use the Add method to add data in their UpdateLogData method as Name/Value pairs
// Internally, this class holds the Names & Values in 2 arrays
// Therefore this class does not need to be changed to support addl data to be logged, it grows dynamically
public class LogDataBE {
	// define class level working variables
	private List<String> _names;
	private List<String> _values;
	
	public LogDataBE() {
		_names = new ArrayList<String>();
		_values = new ArrayList<String>();
	}
		
	public void AddData(String name, String value) {
		_names.add(name);
		_values.add(value);
	}
	
	/** Discard any data currently being held */
	public void ResetData() {
		_names.clear();
		_values.clear();	
	}

	/** Build a TSV (tab separated value) string for the header row */
	public String BuildTSVHeader() {
		return BuildTSVString(_names);
	}

	/** Build a TSV (tab separated value) string for a data row */
	public String BuildTSVData() {
		return BuildTSVString(_values);
	}
	
	/** Build a TSV string from a List<string> */
	private String BuildTSVString(List<String> myList) {
		StringBuilder sb = new StringBuilder();
		
		for(String item : myList) {
			// add the item + a tab character
			sb.append(item + "\t");
		}
		
		String lineToWrite = sb.toString();
		
		// add trailing crlf
		lineToWrite = lineToWrite + "\r\n";
		
		return lineToWrite;
	}
}