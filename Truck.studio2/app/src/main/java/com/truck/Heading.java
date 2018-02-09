/*
 * Truck
 * Copyright (C) 2018  Adam Williams <broadcast at earthling dot net>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 * 
 */

// Compass for heading

package com.truck;

import java.util.Formatter;

import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.Rect;
import android.util.Log;

public class Heading extends Container
{
	public Heading(int x,
		int y,
		int w, 
		int h)
	{
		super(x, y, w, h);

		center_x = this.x + this.w / 2;
		center_y = this.y + this.h / 2;
	}


	public void draw_arrow(Canvas g, Paint p, float value, boolean filled)
	{
// Get coords
		int[] x = new int[arrow.length];
		int[] y = new int[arrow.length];
		int radius = this.w / 2 - 
			(int)Truck.settings.small_font_size -
			5;

// Rotate shape & scale
		for(int i = 0; i < arrow.length; i++)
		{
			double[] point = arrow[i];
			double r = Math.sqrt(point[0] * point[0] + point[1] * point[1]);
			double angle = Math.atan2(point[1], point[0]);
			angle += value;
			x[i] = (int)(radius * r * Math.cos(angle)) + center_x;
			y[i] = (int)(radius * r * Math.sin(angle)) + center_y;
		}

// Draw it
//		if(filled)
//			g.fillPolygon(x, y, arrow.length);
//		else
//			g.drawPolygon(x, y, arrow.length);
	}

	public void draw(Canvas g, Paint p)
	{
//System.out.println("CopterMainWindow.paint 1");
//		g.setColor(Truck.settings.foreground);
//
//		draw_arrow(g, p, value, true);
//		draw_arrow(g, p, target_value, false);
//
//// Draw text
//		p.setTypeface(Truck.settings.small_font);
//		p.setTextSize(Truck.settings.small_font_size);
//		Rect text_size = new Rect();
//		p.getTextBounds("N",
//				0,
//				1,
//				text_size);
//		g.drawString("N",
//			center_x - (int)text_size.width() / 2,
//			this.y + (int)Settings.small_font_size);
//
//		text_size = mwindow.small_font.getStringBounds("S",
//           0,
//           1,
//           frc);
//		g.drawString("S",
//			center_x - (int)text_size.getWidth() / 2,
//			this.y + this.h);
//
//// 		text_size = mwindow.small_font.getStringBounds("E",
////            0,
////            1,
////            frc);
//// 		g.drawString("E",
//// 			this.x + this.w - (int)text_size.getWidth(),
//// 			center_y + (int)mwindow.small_font_size.getHeight() / 2);
////
//// 		g.drawString("W",
//// 			this.x,
//// 			center_y + (int)mwindow.small_font_size.getHeight() / 2);
//		g.drawOval(this.x, this.y, this.w, this.h);
	}

	public void set_value(float value)
	{
		//this.value = Math2.toRad((double)value);
	}

	public void set_target(float value)
	{
		//this.target_value = Math2.toRad((double)value);
	}

	int x;
	int y;
	int w;
	int h;

// Points in shape relative to center
	static final double[][] arrow = 
	{
		{ 0,     -1 },
		{ 0.25,  -0.75 },
		{ 0.25,  1 },
		{ -0.25, 1 },
		{ -0.25, -0.75 }
	};
	
	float value;
	float target_value;
	int center_x;
	int center_y;
}


