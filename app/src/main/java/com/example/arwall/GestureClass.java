package com.example.arwall;

import android.app.Activity;
import androidx.core.view.GestureDetectorCompat;
import android.view.GestureDetector;
import android.view.MotionEvent;

public class GestureClass {

    GestureDetectorCompat mTapDetector;
    private native void DoubleTapNative();

    public GestureClass(Activity activity) {

        // instantiate listener for detecting double-tap
        mTapDetector = new GestureDetectorCompat(activity, new MyTapListener());
    }

    // this class detects double-tap gesture
    class MyTapListener extends GestureDetector.SimpleOnGestureListener {

        public boolean onDoubleTap (MotionEvent event) {
            DoubleTapNative();
            return true;
        }
    }
}
