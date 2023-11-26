package com.truck;

import java.util.Vector;

import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.Rect;

public class SmallReadout extends Container
{
	int total;
	int[] positions;
	String[] values;
	String title;
    int title_color = Truck.settings.foreground;
    int value_color = Truck.settings.foreground;


	public SmallReadout(int x, int y, int total)
	{
		super(x, y, 0, 0);
		this.total = total;
		values = new String[total];
		positions = new int[total];
	}
	
	
	public void draw(Canvas g, Paint p)
	{
		p.setColor(title_color);
        p.setStyle(Paint.Style.FILL);
		p.setTypeface(Truck.settings.small_font);
		p.setTextSize(Truck.settings.small_font_size);
		Rect textSize = new Rect();
		p.getTextBounds("()",
		           0,
		           2,
		           textSize);

		int x = this.x;
		int y = this.y + textSize.height();
		if(title != null)
		{
			g.drawText(title, x, y, p);
		}
		
		p.setColor(value_color);
		for(int i = 0; i < total; i++)
		{
			int x2 = x + positions[i];
			if(values[i] != null) g.drawText(values[i], x2, y, p);
		}
	}
	
	public void update(int slot, String value)
	{
		values[slot] = value;
	}
	
	public void setTitle(String value)
	{
		title = value;
	}
	
	public void setPositions(int[] positions)
	{
		this.positions = positions;
	}
	
	public int getH(Paint p)
	{
		p.setTypeface(Truck.settings.small_font);
		p.setTextSize(Truck.settings.small_font_size);
		Rect textSize = new Rect();
		p.getTextBounds("()",
		           0,
		           2,
		           textSize);
		return textSize.height();
	}
}
