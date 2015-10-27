package com.truck;

import com.truck.R;
import java.util.Formatter;
import android.os.Bundle;
import android.view.View;
import android.widget.CheckBox;
import android.widget.EditText;
import android.widget.SeekBar;
import android.widget.TextView;

public class SettingsWin  extends WindowBase implements SeekBar.OnSeekBarChangeListener
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

    public void onClick(View view)
    {
        switch (view.getId())
  		{
        case R.id.send:
        	Settings.loadFile();
			Truck.needSaveConfig = true;
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
        	Truck.needReset = true;
        	break;
  		}
  	}

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
