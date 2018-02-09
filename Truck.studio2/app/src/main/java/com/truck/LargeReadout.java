package com.truck;

import java.util.Formatter;

import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.Rect;
import android.util.Log;

public class LargeReadout extends Container {
	public LargeReadout(int x, 
			int y,
			String title,
			String value)
	{
		super(x, y, 0, 0);
		this.title = title;
		this.value = value;
	}
	
	public void draw(Canvas g, Paint p)
	{
//System.out.println("CopterMainWindow.paint 1");
		p.setColor(Truck.settings.foreground);
        p.setStyle(Paint.Style.FILL);

		int x = this.x;
		int y = this.y;
		int w = this.w;
		
		if(this.w == 0) 
		{
			w = getW(p);
			this.w = w;
		}
		int centerX = x + w / 2;

		p.setTypeface(Truck.settings.small_font);
		p.setTextSize(Truck.settings.small_font_size);
		Rect text_size = new Rect();
		p.getTextBounds(title,
		           0,
		           title.length(),
		           text_size);
		y += text_size.height();
		g.drawText(title, centerX - text_size.width() / 2, y, p);

		p.setTypeface(Truck.settings.big_font);
		p.setTextSize(Truck.settings.big_font_size);
		p.getTextBounds(value,
		           0,
		           value.length(),
		           text_size);
		y += text_size.height() + Settings.margin;
		g.drawText(value, centerX  - text_size.width() / 2, y, p);
		
//		Log.v("CopterLargeReadout", "draw " + x + " " + y + " " + value);
	}

	public void setValue(float value)
	{
		StringBuilder sb = new StringBuilder();
		Formatter formatter = new Formatter(sb);
		formatter.format("%.02f", value);
		this.value = formatter.toString();
//		Log.v("CopterLargeReadout", "setValue " + this.value);
	}
	
	public void update(String value)
	{
		this.value = value;
//		Log.v("CopterLargeReadout", "update " + this.value);
	}
	
	public int getW(Paint p)
	{
		Rect text_size = new Rect();

		p.setTypeface(Truck.settings.small_font);
		p.setTextSize(Truck.settings.small_font_size);

		p.getTextBounds(title,
		           0,
		           title.length(),
		           text_size);
		int result = text_size.width();

		p.setTypeface(Truck.settings.big_font);
		p.setTextSize(Truck.settings.big_font_size);
		p.getTextBounds(value,
           0,
           value.length(),
           text_size);
		return Math.max(result, text_size.width());
	}
	
	public int getH(Paint p)
	{
		Rect text_size = new Rect();

		p.setTypeface(Truck.settings.small_font);
		p.setTextSize(Truck.settings.small_font_size);

		p.getTextBounds(title,
		           0,
		           title.length(),
		           text_size);
		int result = text_size.height();

		p.setTypeface(Truck.settings.big_font);
		p.setTextSize(Truck.settings.big_font_size);
		p.getTextBounds(value,
           0,
           value.length(),
           text_size);
		return result + text_size.height() + Settings.margin;
	}

	String title;
	String value;

}






