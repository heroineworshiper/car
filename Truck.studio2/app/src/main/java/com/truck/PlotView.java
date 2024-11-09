package com.truck;


import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Typeface;
import android.util.AttributeSet;
import android.util.Log;
import android.view.View;

import java.util.Formatter;


public class PlotView extends View {

// from plot.py
    final float STEP = 10;
    final int SAMPLERATE = 100;
    final float MIN_FREQ = (float).1;
    final float MAX_FREQ = 10;
    final double[] PRINT_FREQS = { .1, .2, .3, .4, .5, .7, 1, 2, 3, 4, 5, 7, 10 };
    final double[] PRINT_POWERS = { 1, .7, .5, .4, .3, .2, .1 };
    final int ORDER = 2;
    int graph_x;
    int graph_y;
    int graph_w;
    int graph_h;
    float min_x = (float)Math.log10(MIN_FREQ);
    float max_x = (float)Math.log10(MAX_FREQ);
    float min_y = (float)Math.log10(.1);
    float max_y = (float)Math.log10(1);
    float max;
    float min;

    class Filter
    {
        float[] prev_output;
        float[] prev_input;
        float result;
        float bandwidth;
        
        public Filter()
        {
        }
        
        public void init(float bandwidth)
        {
            prev_output = new float[ORDER];
            prev_input = new float[ORDER];
            for(int i = 0; i < ORDER; i++)
            {
                prev_output[i] = 0;
                prev_input[i] = 0;
            }
            result = 0;
            this.bandwidth = bandwidth;
        }

        public float highpass(float value)
        {
            result = value;
            for(int i = 0; i < ORDER; i++)
            {
                result = bandwidth * (prev_output[i] + value - prev_input[i]);
                prev_input[i] = value;
                prev_output[i] = result;
                value = result;
            }
            return result;
        }

        public float lowpass(float value)
        {
            result = value;
            for(int i = 0; i < ORDER; i++)
            {
                result = prev_output[i] + bandwidth * (value - prev_output[i]);
                prev_input[i] = value;
                prev_output[i] = result;
                value = result;
            }
            return result;
        }
    }
    Filter leash_lowpass = new Filter();
    Filter leash_lowpass2 = new Filter();
    Filter leash_highpass = new Filter();
    float leash_p;
    float leash_p2;


    public PlotView(Context context, AttributeSet attrs) {
        super(context, attrs);
    }

    public PlotView(Context context, AttributeSet attrs, int default_style) {
        super(context, attrs, default_style);
    }

    public PlotView(Context context) {
        super(context);
    }

    float freq_to_x(float f)
    {
        return (float)((Math.log10(f) - min_x) / (max_x - min_x) * graph_w);
    }


    float x_to_freq(float x)
    {
        float x3 = x / graph_w * (max_x - min_x) + min_x;
        return (float) Math.pow(10, x3);
    }

    float power_to_y(float v)
    {
        return (float)((Math.log10(v) - min_y) / (max_y - min_y) * graph_h);
    }

    float leash_function(float v)
    {
        float low = leash_lowpass.lowpass(v);
        float low2 = leash_lowpass2.lowpass(v);
        float high = leash_highpass.highpass(low2);
        float result = low * leash_p + high * leash_p2;
        return result;
    }

    float compute_power(float freq)
    {
// how many periods to sample
        final int PERIODS = 100;
        final int samples = (int)(SAMPLERATE * PERIODS / freq);
        float max = 0;
        for(int i = 0; i < samples; i++)
        {
            float input = (float)Math.sin((float)i / SAMPLERATE * 2 * Math.PI * freq);
            float output = leash_function(input);
            float abs_output = Math.abs(output);
// drop 1st period & get maximum of remaneing samples
            if(i >= SAMPLERATE / freq && abs_output > max)
                max = abs_output;
        }
//        return (float)Math.log10(max);
        return max;
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);

        Paint paint = new Paint();
        paint.setDither(true);
        paint.setColor(Color.BLACK);
        paint.setStyle(Paint.Style.FILL);
        paint.setStrokeWidth(1);
        paint.setTypeface(Typeface.create("SansSerif", Typeface.NORMAL));
        paint.setTextSize(24);

        graph_x = 50;
        graph_y = 20;
        graph_w = canvas.getWidth() - graph_x;
        graph_h = canvas.getWidth();
        
        max = -99999;
        min = 99999;

        // reload the values
        Truck.settings.loadFile();
        // reset the filters
        leash_lowpass.init(Truck.settings.getFileFloat("LEASH_LOWPASS")[0] / 100);
        leash_lowpass2.init(Truck.settings.getFileFloat("LEASH_LOWPASS2")[0] / 100);
        leash_highpass.init(Truck.settings.getFileFloat("LEASH_HIGHPASS")[0] / 100);

        float pid[] = Truck.settings.getFileFloat("LEASH_PID");
        leash_p = pid[0];
        leash_p2 = pid[3];
        Log.i("x", "PlotView.onDraw canvas=" + canvas +
            " w=" + graph_w + 
            " h=" + graph_h + 
            " p=" + leash_p +
            " p2=" + leash_p2 + 
            " low=" + leash_lowpass.bandwidth +
            " low2=" + leash_lowpass2.bandwidth +
            " high=" + leash_highpass.bandwidth);


        compute_power(MIN_FREQ);
        float[] buffer = new float[graph_w + 1];
        for(float x = 0; x < graph_w; x += STEP)
        {
            float freq = x_to_freq(x);
            buffer[(int)x] = compute_power(freq);
            if(buffer[(int)x] > max)
                max = buffer[(int)x];
            if(buffer[(int)x] < min)
                min = buffer[(int)x];
            //Log.i("x", "PlotView.onDraw freq=" + freq + 
            //    " value=" + buffer[(int)x]);
        }
        
        

// frequency reticules
        for(int i = 0; i < PRINT_FREQS.length; i++)
        {
            float x = freq_to_x((float)PRINT_FREQS[i]) + graph_x;
            //Log.i("x", "PlotFilter.surfaceCreated x=" + x);
            canvas.drawLine(x, graph_y, x, graph_y + graph_h, paint);

            StringBuilder sb = new StringBuilder();
            Formatter formatter = new Formatter(sb);
            if(PRINT_FREQS[i] < 1)
                formatter.format(".%.0f", PRINT_FREQS[i] * 10);
            else
                formatter.format("%.0f", PRINT_FREQS[i]);
            if(i == PRINT_FREQS.length - 1)
                x -= 25;
            else
                x -= 10;
            canvas.drawText(sb.toString(), 
                0, 
                sb.toString().length(), 
                x, 
                graph_y + graph_h + 20, 
                paint);
        }
        
        
//         for(int i = 0; i < PRINT_POWERS.length; i++)
//         {
//             float y = graph_y + graph_h - power_to_y((float)PRINT_POWERS[i]);
//             canvas.drawLine(graph_x, 
//                 y, 
//                 graph_x + graph_w, 
//                 y, 
//                 paint);
//             StringBuilder sb = new StringBuilder();
//             Formatter formatter = new Formatter(sb);
//             formatter.format("%.1f", PRINT_POWERS[i] * (max - min) + min);
//             if(i == 0)
//                 y += 20;
//             canvas.drawText(sb.toString(), 
//                 10, 
//                 y, 
//                 paint);
//         }
        
        for(int i = 0; i < 11; i++)
        {
            float y = graph_y + graph_h - i * graph_h / 10;
            canvas.drawLine(graph_x, 
                y, 
                graph_x + graph_w, 
                y, 
                paint);
            StringBuilder sb = new StringBuilder();
            Formatter formatter = new Formatter(sb);
            formatter.format("%.0f", i * (max - min) / 10 + min);
            canvas.drawText(sb.toString(), 
                5, 
                y + 5, 
                paint);
        }

        float x1 = -STEP;
        float y1 = -1;
        paint.setStrokeWidth(2);
        paint.setColor(Color.RED);
        for(float x = 0; x < graph_w; x += STEP)
        {
            float x2 = x1 + STEP;
            float y2 = graph_y + graph_h - 1 - (buffer[(int)x] - min) / (max - min) * (graph_h - 2);
            if(x1 >= 0)
                canvas.drawLine(graph_x + x1, 
                    y1, 
                    graph_x + x2, 
                    y2,
                    paint);
            x1 = x2;
            y1 = y2;
        }
    }
}










