package com.truck;

import com.truck.R;
import java.util.Formatter;
import android.os.Bundle;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.CheckBox;
import android.widget.EditText;
import android.widget.SeekBar;
import android.widget.Spinner;
import android.widget.TextView;

public class SettingsWin  extends WindowBase implements SeekBar.OnSeekBarChangeListener, AdapterView.OnItemSelectedListener
{
	@Override
	public void onCreate(Bundle savedInstanceState) {
	    super.onCreate(savedInstanceState);
        setContentView(R.layout.settings);
        Truck.createObjects(this);
        
        

		SeekBar sb = (SeekBar)findViewById(R.id.pace);
		sb.setProgress(paceToSlider(Settings.targetPace));
		sb.setOnSeekBarChangeListener(this);

		updatePaceText();

		TextView text = (TextView)findViewById(R.id.message);
		text.setText(Settings.message);


		ArrayAdapter<CharSequence> adapter = ArrayAdapter.createFromResource(this,
                R.array.vehicle_options,
                R.layout.spinner_layout);
		Spinner vehicle = (Spinner)findViewById(R.id.vehicle);
		vehicle.setAdapter(adapter);
		vehicle.setOnItemSelectedListener(this);
		vehicle.setSelection(Settings.vehicle);


		//CheckBox checkbox;
        //checkbox = (CheckBox) findViewById(R.id.headlights);
        //checkbox.setChecked(Settings.headlights);
        updateGUI();
	}

	public void updatePaceText()
	{
		TextView text = (TextView)findViewById(R.id.pace_text);
		text.setText(new Formatter().format("%.1f min/mile", Settings.targetPace).toString());

	}

	public int paceToSlider(float pace)
	{
		return (int)((pace - 5) * 2);
	}

	public float sliderToPace(int slider)
	{
		return ((float)slider / 2) + 5;
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

		synchronized(Truck.truck)
		{
			if(Settings.haveMessage)
			{
				TextView text = (TextView)findViewById(R.id.message);
				text.setText(Settings.message);

				Settings.haveMessage = false;
			}
		}

	}
	
	
	@Override
	public void onItemSelected(AdapterView<?> parent, 
			View view, 
            int pos, 
            long id) 
	{
    	switch (parent.getId()){
    	case R.id.vehicle:
			Settings.vehicle = pos;
			
			if(Settings.vehicle != Settings.prevVehicle)
			{
				// Load the new settings, but user should restart anyway.
				Settings.loadFile();
				updatePaceText();
//				Truck.truck.initializeBluetooth();
// force it to reconnect without crashing the mane loop
				Truck.truck.vehicleChanged = true;
			}

    		Settings.save();
    		break;
    	}
    	
	}

	@Override
	public void onNothingSelected(AdapterView<?> arg0) 
	{
	}


    public void onClick(View view)
    {
        switch (view.getId())
  		{
        case R.id.send:
        	Settings.loadFile();
//			Truck.needSaveConfig = true;
        	Truck.needConfig = true;
        	break;
//        case R.id.headlights:
//        	Settings.loadFile();
//        	Settings.headlights = !Settings.headlights;
//        	Settings.save();
//        	Truck.needConfig = true;
//        	Truck.needSaveConfig = true;
//        	break;
        case R.id.reset:
            Truck.confirm(this, Truck.RESET_COMMAND);
//        	Truck.needReset = true;
        	break;
        case R.id.test_motors:
            Truck.confirm(this, Truck.TEST_MOTORS_COMMAND);
//        	Truck.testMotors = true;
        	break;
  		}
  	}

// user changed target speed
	@Override
	public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser)
	{
		Settings.targetPace = sliderToPace(progress);
		updatePaceText();

		Settings.save();
	}

	@Override
	public void onStartTrackingTouch(SeekBar seekBar) {

	}

	@Override
	public void onStopTrackingTouch(SeekBar seekBar) {

	}
    

}
