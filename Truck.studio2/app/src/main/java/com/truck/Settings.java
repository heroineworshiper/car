/*
 * Phone app for direct drive truck
 * Copyright (C) 2012-2023 Adam Williams <broadcast at earthling dot net>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 * 
 */


package com.truck;


import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.Formatter;
import java.util.StringTokenizer;
import java.util.Vector;

import android.app.Activity;
import android.content.Context;
import android.content.SharedPreferences;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Typeface;
import android.os.Environment;
import android.util.Log;


// global settings
public class Settings {
	public Settings(Activity activity)
	{
		context = activity.getApplicationContext();

		load();
		loadFile();
		
		

		big_font = Typeface.create("SansSerif", Typeface.BOLD);
		small_font = Typeface.create("SansSerif", Typeface.BOLD);
		big_font_size = 70;
		small_font_size = 36;
		foreground = Color.WHITE;
		background = Color.BLACK;
	}

	
	static void loadFile()
	{
// load internal guidance parameters from file
		File dir = new File(DIR);
		File file = null;

		switch(vehicle)
		{
			case TRUCK:
				file = new File(DIR + TRUCK_FILENAME);
				break;
			
			case TRUCKCAM:
				file = new File(DIR + TRUCKCAM_FILENAME);
				break;
			
			case CAR:
			default:
				file = new File(DIR + CAR_FILENAME);
				break;
			
		}
		
		keys.clear();
		stringValues.clear();
		
		
// make the directory if it doesn't exist
        try {
            boolean result = dir.mkdirs();
        }
        catch(SecurityException e) {
            Log.v("Settings", "loadInternal 1 " + e.toString());
        }

        
        BufferedReader reader = null;
		try {
			reader = new BufferedReader(new FileReader(file));
		} catch (FileNotFoundException e) {
            Log.i("Settings", "loadInternal 2 " + e.toString());
            return;
		}
        String line = null;
        try {
			while ((line = reader.readLine()) != null) 
			{
//				Log.v("Settings", "loadInternal " + line);
				
				StringTokenizer st = new StringTokenizer(line);
				Vector<String> strings = new Vector<String>();
				while(st.hasMoreTokens())
				{
					strings.add(st.nextToken());
				}

				// comment or whitespace			
				if(strings.size() == 0 || strings.get(0).charAt(0) == '#') continue;

				keys.add(strings.get(0));
				String[] newStringValues = new String[strings.size() - 1];
				for(int i = 0; i < strings.size() - 1; i++)
				{
					newStringValues[i] = strings.get(i + 1);
				}
				stringValues.add(newStringValues);

			}
		} catch (IOException e) {
            Log.v("Settings", "loadInternal " + e.toString());
		}
        
        try {
			reader.close();
		} catch (IOException e) {
            Log.v("Settings", "loadInternal " + e.toString());
		}
	
        dumpFile();

        
		bluetooth_id = getFileString("BLUETOOTH", bluetooth_id);
        writeDebug = ((int)getFileFloat("DEBUG")[0] == 1);
        Log.i("x", "Settings.loadFile writeDebug=" + getFileFloat("DEBUG")[0]);
	}

	void load()
	{
		SharedPreferences file = null;
		file = context.getSharedPreferences("truck", 0);
		//headlights = file.getInt("headlights", 0) == 0 ? false : true;
		targetPace = file.getFloat("targetPace", targetPace);
		vehicle = file.getInt("vehicle", vehicle);
		prevVehicle = vehicle;
	}
	
	static void save()
	{
		SharedPreferences file2 = null;
		SharedPreferences.Editor file = null;
		file2 = context.getSharedPreferences("truck", 0);
		file = file2.edit();
		
		//file.putInt("headlights", headlights ? 1 : 0);
		file.putFloat("targetPace", targetPace);
		file.putInt("vehicle", vehicle);
		prevVehicle = vehicle;

		file.commit();
	}
	
	// get value from user file
	static public float[] getFileFloat(String key)
	{
		for(int i = 0; i < keys.size(); i++)
		{
			if(key.toUpperCase().equals(keys.get(i).toUpperCase()))
			{
				String[] values = stringValues.get(i);
				float[] newValues = new float[values.length];
				for(int j = 0; j < values.length; j++)
				{
					newValues[j] = Float.parseFloat(values[j]);
				}

				return newValues;
			}
		}
		
		Log.v("Settings", "getInternal " + key + " not found total=" + keys.size());
		float[] dummy = new float[] { 0, 0, 0, 0, 0, 0, 0, 0 };
		return dummy;
	}

	static public String getFileString(String key, String default_)
	{
		for(int i = 0; i < keys.size(); i++)
		{
			if(key.toUpperCase().equals(keys.get(i).toUpperCase()))
			{
				return stringValues.get(i)[0];
			}
		}
		
		Log.v("Settings", "getInternalString " + key + " not found total=" + keys.size());
		return default_;
	}

	
	static public void dumpFile()
	{
		Log.v("Settings", "dumpFile total=" + keys.size());			
		for(int i = 0; i < keys.size(); i++)
		{
			StringBuilder sb = new StringBuilder();
			Formatter formatter = new Formatter(sb);
			formatter.format("%s ", keys.get(i));
			for(int j = 0; j < stringValues.get(i).length; j++) 
			{
				formatter.format("%s ", stringValues.get(i)[j]);
			}
			Log.v("Settings", "dumpInternal " + sb.toString());			
		}
	}

	static boolean initCanvas(Canvas canvas)
	{
		if(!canvasInitialized)
		{
			canvasInitialized = true;
	
			int largeDimension = Math.max(canvas.getWidth(), canvas.getHeight());
			// set up extents
			Settings.border = largeDimension / 40;
			Settings.margin = largeDimension / 40;

			if(Settings.big_font_size == 0)
			{
				Settings.big_font_size = largeDimension / 6;
				Settings.small_font_size = largeDimension / 18;
			}
			
			Settings.screenW = canvas.getWidth();
			Settings.screenH = canvas.getHeight();
			return true;
		}
		
		return false;
	}
	
	
	static final String DIR = "//sdcard//truck//";
	static final String CAR_FILENAME = "settings.car";
	static final String TRUCK_FILENAME = "settings.truck";
	static final String TRUCKCAM_FILENAME = "settings.truckcam";
	static final String DEBUG_FILENAME = "debug";
	static boolean writeDebug = false;
	static String bluetooth_id = "truck";
//	static boolean headlights = false;
	static float targetPace = 9.00f;
	
	static Context context;
	

	// period in milliseconds to update GUI
	static int gui_dt = 33;
	
	// GUI sizes
	// distance between widgets
	static int margin = 0;
	// distance from edge of screen
	static int border = 0;
	

	// constants
	// largest packet
	static final int RADIO_BUFSIZE = 1024;
	static final int BEACON_HZ = 10;

	static boolean canvasInitialized = false;
	static Typeface big_font;
	static Typeface small_font;
	static int big_font_size = 0;
	static int small_font_size = 0;
	static int foreground;
	static int background;
	static int screenW;
	static int screenH;
	static int stickW;
// message to display
	static String message = null;
	static boolean haveMessage = false;

// the supported vehicles
	static final int TRUCK = 0;
	static final int CAR = 1;
	static final int TRUCKCAM = 2;
	static int vehicle = TRUCK;
	static int prevVehicle = vehicle;


	static Vector<String> keys = new Vector<String>();
	static Vector<String[]> stringValues = new Vector<String[]>();
}
