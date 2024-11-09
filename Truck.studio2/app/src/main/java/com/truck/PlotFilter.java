package com.truck;

import java.util.Formatter;
import java.util.Vector;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.PorterDuff;
import android.graphics.Typeface;
import android.os.Bundle;
import android.os.Message;
import android.os.Vibrator;
import android.util.Log;
import android.view.MotionEvent;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;
import android.view.View.OnTouchListener;
import android.widget.TextView;


// draw the leash filter

public class PlotFilter extends WindowBase
{
    PlotView canvas;

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.plot_filter);
        Truck.createObjects(this);
        canvas = (PlotView)findViewById(R.id.plot_view);
    }

    public void onClick(View view)
    {
        switch (view.getId())
  		{
        case R.id.reload:
            canvas.postInvalidate();
            break;
        }
    }

}




