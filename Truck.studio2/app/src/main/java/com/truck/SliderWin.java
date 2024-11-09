package com.truck;

import java.util.Vector;

import android.content.Context;
import android.os.Bundle;
import android.os.Message;
import android.util.Log;
import android.view.View;
import android.widget.SeekBar;



public class SliderWin extends WindowBase implements SeekBar.OnSeekBarChangeListener
{
    SeekBar steering;
    SeekBar leftMotor;
    SeekBar rightMotor;
    final int MID = 50;
    final int MAG = 50;

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.sliders);
        Truck.createObjects(this);



      	steering = (SeekBar)findViewById(R.id.steering_slider);
		steering.setProgress(50);
		steering.setOnSeekBarChangeListener(this);

      	leftMotor = (SeekBar)findViewById(R.id.motor0_slider);
		leftMotor.setProgress(50);
		leftMotor.setOnSeekBarChangeListener(this);

      	rightMotor = (SeekBar)findViewById(R.id.motor1_slider);
		rightMotor.setProgress(50);
		rightMotor.setOnSeekBarChangeListener(this);
        
        Truck.steeringOut = 0;
        Truck.throttleOut[0] = 0;
        Truck.throttleOut[1] = 0;
        Truck.haveControls = true;
    }


    public void onPause()
    {
        super.onPause();
        synchronized(Truck.truck)
        {
            Truck.haveControls = false;
        }

    }


    public void onStop()
    {
        super.onStop();
        synchronized(Truck.truck)
        {
            Truck.haveControls = false;
        }
    }


    public void onClick(View view)
    {
        switch (view.getId())
  		{
            case R.id.reset_steering:
                Truck.steeringOut = 0;
                steering.setProgress(50);
                break;
            case R.id.reset_left:
                Truck.throttleOut[0] = 0;
                leftMotor.setProgress(50);
                break;
            case R.id.reset_right:
                Truck.throttleOut[1] = 0;
                rightMotor.setProgress(50);
                break;
  		}
  	}



	@Override
	public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser)
	{
//		Log.i("SliderWin", "onProgressChanged seekBar=" + seekBar + " progress=" + progress);
	
        if(seekBar == steering)
        {
//            Log.i("SliderWin", "onProgressChanged steering");
            Truck.steeringOut = (progress - MID) * 127 / MAG;
        }

        if(seekBar == leftMotor)
        {
//            Log.i("SliderWin", "onProgressChanged steering");
            Truck.throttleOut[0] = (progress - MID) * 127 / MAG;
        }

        if(seekBar == rightMotor)
        {
//            Log.i("SliderWin", "onProgressChanged steering");
            Truck.throttleOut[1] = (progress - MID) * 127 / MAG;
        }
    }

	@Override
	public void onStartTrackingTouch(SeekBar seekBar) {

	}

	@Override
	public void onStopTrackingTouch(SeekBar seekBar) {

	}
}
