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
		small_font_size = 24;
		foreground = Color.WHITE;
		background = Color.BLACK;
	}

	
	void loadFile()
	{
// load internal guidance parameters from file
		File dir = new File(Environment.getExternalStorageDirectory() + DIR);
		File file = new File(Environment.getExternalStorageDirectory() + DIR + FILENAME);

		
		
// make the directory if it doesn't exist
        try {
            boolean result = dir.mkdirs();
        }
        catch(SecurityException e) {
            Log.v("Settings", "loadInternal " + e.toString());
        }

        
        BufferedReader reader = null;
		try {
			reader = new BufferedReader(new FileReader(file));
		} catch (FileNotFoundException e) {
            Log.v("Settings", "loadInternal 2 " + e.toString());
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
	}

	void load()
	{
		SharedPreferences file = null;
		file = context.getSharedPreferences("truck", 0);

	}
	
	void save()
	{
		SharedPreferences file2 = null;
		SharedPreferences.Editor file = null;
		file2 = context.getSharedPreferences("truck", 0);
		file = file2.edit();


		file.commit();
	}
	
	// get value to be uploaded
	public float[] getFileFloat(String key)
	{
		for(int i = 0; i < keys.size(); i++)
		{
			if(key.toUpperCase().equals(keys.get(i).toUpperCase()))
			{
				String[] values = stringValues.get(i);
				float[] newValues = new float[stringValues.size() - 1];
				for(int j = 0; j < values.length; j++)
				{
					newValues[j] = Float.parseFloat(values[j]);
				}

				return newValues;
			}
		}
		
		Log.v("Settings", "getInternal " + key + " not found total=" + keys.size());
		float[] dummy = new float[1];
		dummy[0] = 0;
		return dummy;
	}

	public String getFileString(String key, String default_)
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

	
	public void dumpFile()
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
	
	
	
	static final String DIR = "//truck//";
	static final String FILENAME = "settings.conf";
	static String bluetooth_id = "Truck";
	
	
	Context context;
	

	// period in milliseconds to update GUI
	static int gui_dt = 33;
	
	// GUI sizes
	// distance between widgets
	static int margin = 0;
	// distance from edge of screen
	static int border = 0;
	static int stickW;
	static int screenW;
	

	// constants
	// largest return packet
	static final int RADIO_BUFSIZE = 128;
	// ground beacon for copter
	static final int GROUND_RADIO_OUT_SIZE = 0x20;
	static final byte SYNC_CODE = (byte) 0xe5;
	static final int CONFIG_SECTIONS = 4;
	
	static final int ENABLE_POV_BIT = 3;
	static final int ENGINE_ON_BIT = 4;
	static final int RADIO_VALID_BIT = 5;
	static final int CONFIG_VALID_BIT = 6;
	
	static final byte PACKET_CONFIG = 3 << 4;
	static final byte PACKET_AZIMUTH = (byte) (9 << 4);
	static final byte PACKET_ANALOG = 0;
	
	static final int CONFIG_PACKET_SIZE = 6;
	static final int AZIMUTH_SEND_SAMPLES = 8;
	static final int AZIMUTH_SAMPLE_BYTES = 2;
	static final int AZIMUTH_PACKET_SIZE = 1 + 1 + 2 + 2 + AZIMUTH_SEND_SAMPLES * AZIMUTH_SAMPLE_BYTES + 2;
	static final int ANALOG_PACKET_SIZE = 4 + 4 * 2;
	// 0xff, 0x2d, 0xd4, packet size from mrf49xa usage
	static final int HEADER_SIZE = 4;
	static final int BEACON_HZ = 10;

	
	static Typeface big_font;
	static Typeface small_font;
	static int big_font_size;
	static int small_font_size;
	static int foreground;
	static int background;
	static int waypointColor = Color.RED;
	
	

	Vector<String> keys = new Vector<String>();
	Vector<String[]> stringValues = new Vector<String[]>();
}
