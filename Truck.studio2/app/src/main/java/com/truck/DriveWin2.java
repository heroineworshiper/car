package com.truck;

import java.util.Vector;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.PorterDuff;
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


// semi autonomous controller

public class DriveWin2 extends WindowBase implements OnTouchListener
{

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.drive2);
        Truck.createObjects(this);

        SurfaceView canvas = (SurfaceView) findViewById(R.id.canvas2);
        SurfaceHolder mSurfaceHolder = canvas.getHolder();
        canvas.setOnTouchListener(this);

        paint = new Paint();
        paint.setDither(true);
        paint.setColor(Color.WHITE);
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeJoin(Paint.Join.ROUND);
        paint.setStrokeCap(Paint.Cap.ROUND);
        paint.setStrokeWidth(8);
    }


    public void onPause()
    {
        super.onPause();
        synchronized(Truck.truck)
        {
            Truck.haveControls2 = false;
        }

    }


    public void onStop()
    {
        super.onStop();
        synchronized(Truck.truck)
        {
            Truck.haveControls2 = false;
        }
    }

    public void updateGUI()
    {
        SurfaceView canvas1 = (SurfaceView) findViewById(R.id.canvas2);
        SurfaceHolder mSurfaceHolder = canvas1.getHolder();
        Canvas canvas = mSurfaceHolder.lockCanvas(null);
        if(canvas != null)
        {
            Settings.initCanvas(canvas);
            int stickW = canvas.getWidth();
            int stickH = canvas.getHeight();

            if(stick == null)
            {
                readouts.add(stick = new StickPanel(
                        0,
                        0,
                        stickW,
                        stickH));

            }

            synchronized(Truck.truck)
            {
                if(stick.userY > 0.25)
                {
                    Truck.throttleOut2 = true;
                    Truck.reverse = false;
                }
                else
                if(stick.userY < -0.25)
                {
                    Truck.throttleOut2 = true;
                    Truck.reverse = true;
                }
                else
                {
                    Truck.throttleOut2 = false;
                }

//Log.i("DriveWin2", "userX=" + stick.userX + " userY=" + stick.userY);
                if(stick.userX < -0.6)
                {
                    Truck.steeringOut2 = Truck.FAST_LEFT;
                }
                else
                if(stick.userX < -0.25)
                {
                    if(Truck.steeringOut2 == Truck.FAST_LEFT) {
                        Truck.steeringOut2 = Truck.NO_STEERING;
                        ignoreSlow = true;
                    }
                    else if(!ignoreSlow) {
                        Truck.steeringOut2 = Truck.SLOW_LEFT;
                    }
                }
                else
                if(stick.userX > 0.6)
                {
                    Truck.steeringOut2 = Truck.FAST_RIGHT;
                }
                else
                if(stick.userX > 0.25)
                {
                    if(Truck.steeringOut2 == Truck.FAST_RIGHT)
                    {
                        Truck.steeringOut2 = Truck.NO_STEERING;
                        ignoreSlow = true;
                    }
                    else if(!ignoreSlow){
                        Truck.steeringOut2 = Truck.SLOW_RIGHT;
                    }
                }
                else
                {
                    Truck.steeringOut2 = Truck.NO_STEERING;
                    ignoreSlow = false;
                }

                Truck.haveControls2 = stick.isActive || (activeCountdown > 0);

                // resend a few times after stopping to send the stop command
                if(stick.isActive)
                {
                    activeCountdown = 8;
                }
                else
                if(activeCountdown > 0)
                {
                    activeCountdown--;
                }


                if(Settings.haveMessage)
                {
                    TextView text = (TextView)findViewById(R.id.messages3);
                    if(text != null) text.setText(Settings.message);
                    Settings.haveMessage = false;
                }

            }


            // haptic feedback
            if(Truck.reverse != prevReverse ||
                    Truck.throttleOut2 != prevThrottle ||
                    Truck.steeringOut2 != prevSteering)
            {
                prevReverse = Truck.reverse;
                prevThrottle = Truck.throttleOut2;
                prevSteering = Truck.steeringOut2;

                if(stick.isActive) {
                    Vibrator vibrator = (Vibrator) getSystemService(Context.VIBRATOR_SERVICE);
                    if (vibrator != null) {
                        vibrator.vibrate(50);
                    }
                }
            }

            canvas.drawColor(0, PorterDuff.Mode.CLEAR);
            for (int i = 0; i < readouts.size(); i++) {
                ((Container) readouts.get(i)).draw(canvas, paint);
            }
            mSurfaceHolder.unlockCanvasAndPost(canvas);
        }
    }

    @Override
    public boolean onTouch(View view, MotionEvent motionEvent)
    {
        boolean gotIt = false;
        int pointers = motionEvent.getPointerCount();
        int actionPointerId = (motionEvent.getAction() & MotionEvent.ACTION_POINTER_ID_MASK) >>
                MotionEvent.ACTION_POINTER_ID_SHIFT;
        int actionEnum = motionEvent.getAction() & MotionEvent.ACTION_MASK;


        switch(motionEvent.getAction())
        {
// single point
            case MotionEvent.ACTION_DOWN:
                if(!touch.active &&
                        stick.handlePress(
                                (int)motionEvent.getX(),
                                (int)motionEvent.getY()))
                {
                    touch.active = true;
                    touch.id = motionEvent.getPointerId(0);
                }
                break;

            case MotionEvent.ACTION_UP:
                if(touch.active) stick.handleRelease();
                touch.active = false;
                break;

// multi point
            default:
                for(int i = 0; i < pointers; i++)
                {
                    if(motionEvent.getPointerId(i) == actionPointerId)
                    {
                        switch(actionEnum)
                        {
                            case MotionEvent.ACTION_POINTER_DOWN:
                                if(!touch.active &&
                                        stick.handlePress(
                                                (int)motionEvent.getX(i),
                                                (int)motionEvent.getY(i)))
                                {
                                    touch.active = true;
                                    touch.id = actionPointerId;
                                }
                                break;

                            case MotionEvent.ACTION_POINTER_UP:
                                if(touch.active &&
                                        touch.id == actionPointerId)
                                {
                                    stick.handleRelease();
                                    touch.active = false;
                                }
                                break;
                        }
                    }
                }
                break;

        }



        for(int i = 0; i < pointers; i++)
        {
            if(touch.active &&
                    motionEvent.getPointerId(i) == touch.id)
            {
                stick.handleMotion(
                        (int)motionEvent.getX(i),
                        (int)motionEvent.getY(i));
            }
        }

        return true;
    }



    Vector<Container> readouts = new Vector<Container>();
    TouchPoint touch = new TouchPoint();
    StickPanel stick;
    Paint paint;
    // ignore slow steering when coming back from fast steering
    boolean ignoreSlow = false;
    boolean prevReverse = false;
    boolean prevThrottle = false;
    int prevSteering = Truck.NO_STEERING;
    int activeCountdown = 0;
}
