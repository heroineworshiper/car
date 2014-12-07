package com.truck;

import com.truck.R;

import android.os.Bundle;
import android.view.View;
import android.widget.CheckBox;
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
        CheckBox checkbox;
        checkbox = (CheckBox) findViewById(R.id.headlights);
        checkbox.setChecked(Settings.headlights);
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

    public void onClick(View view)
    {
        switch (view.getId())
  		{
        case R.id.send:
        	Settings.loadFile();
        	Truck.needConfig = true;
        	Truck.needSaveConfig = true;
        	break;
        case R.id.headlights:
        	Settings.loadFile();
        	Settings.headlights = !Settings.headlights;
        	Settings.save();
        	Truck.needConfig = true;
        	Truck.needSaveConfig = false;
        	break;
        case R.id.reset:
        	Truck.needReset = true;
        	break;
  		}
  	}
    
    
	int updateCounter;
}
