package com.truck;

import com.truck.R;

import android.os.Bundle;
import android.widget.EditText;
import android.widget.SeekBar;
import android.widget.TextView;

public class SettingsWin  extends WindowBase
{
	@Override
	public void onCreate(Bundle savedInstanceState) {
	    super.onCreate(savedInstanceState);
        setContentView(R.layout.settings);
        Truck.createObjects(this);
        
        
        updateCounter = 1000 / Settings.gui_dt;
        updateGUI();
	}
	
	public void onPause()
	{
		save();
		super.onPause();
	}

	public void onStop()
	{
		save();
		super.onStop();
	}
 
	void save()
	{
	}

	public void updateGUI()
	{
		updateCounter++;
		if(updateCounter >= 1000 / Settings.gui_dt)
		{
			updateCounter = 0;
			
			
		}
	}

	
	int updateCounter;
}
