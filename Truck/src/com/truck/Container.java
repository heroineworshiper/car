package com.truck;

import android.graphics.Canvas;
import android.graphics.Paint;
import android.view.MotionEvent;
import android.view.View;


// container for a GUI widget
public class Container {
	public Container(int x, 
			int y,
			int w, 
			int h)
	{
		this.x = x;
		this.y = y;
		this.w = w;
		this.h = h;
		center_x = this.x + this.w / 2;
		center_y = this.y + this.h / 2;

	}
	
	
	public void draw(Canvas g, Paint p)
	{
	}

	
	public boolean onTouch(View view, MotionEvent motionEvent) {
		return false;
		
	}


	int x;
	int y;
	int w;
	int h;
	int center_x;
	int center_y;


}
