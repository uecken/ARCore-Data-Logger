package com.pjinkim.arcore_data_logger;

import androidx.appcompat.app.AppCompatActivity;
import androidx.core.content.ContextCompat;

import android.Manifest;
import android.app.Activity;
import android.app.ActivityManager;
import android.content.Context;
import android.content.pm.PackageManager;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.PowerManager;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import com.google.ar.core.ArCoreApk;
import com.google.ar.core.Session;
import com.google.ar.core.TrackingFailureReason;
import com.google.ar.core.TrackingState;
import com.google.ar.core.exceptions.UnavailableApkTooOldException;
import com.google.ar.core.exceptions.UnavailableArcoreNotInstalledException;
import com.google.ar.core.exceptions.UnavailableDeviceNotCompatibleException;
import com.google.ar.core.exceptions.UnavailableSdkTooOldException;
import com.google.ar.core.exceptions.UnavailableUserDeclinedInstallationException;

import java.io.File;
import java.io.IOException;
import java.util.Locale;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.atomic.AtomicBoolean;

public class MainActivity extends AppCompatActivity {

    // properties
    private static final String LOG_TAG = MainActivity.class.getName();
    private static final double MIN_OPENGL_VERSION = 3.0;

    private final static int REQUEST_CODE_ANDROID = 1001;
    private static String[] REQUIRED_PERMISSIONS = new String[] {
            Manifest.permission.CAMERA
            // Removed WRITE_EXTERNAL_STORAGE (deprecated on Android 10+)
            // Removed READ_PHONE_STATE (not essential for ARCore)
            // WAKE_LOCK is not a dangerous permission, doesn't need runtime request
    };

    private ARCoreSession mARCoreSession;
    private Session mSession;
    private boolean mInstallRequested;

    private Handler mHandler = new Handler();
    private AtomicBoolean mIsRecording = new AtomicBoolean(false);
    private PowerManager.WakeLock mWakeLock;

    private TextView mLabelNumberFeatures, mLabelUpdateRate;
    private TextView mLabelTrackingStatus, mLabelTrackingFailureReason;

    private Button mStartStopButton;
    private Button mDetectTagButton;
    private TextView mLabelInterfaceTime;
    private Timer mInterfaceTimer = new Timer();
    private int mSecondCounter = 0;


    // Android activity lifecycle states
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        // check Android and OpenGL version
        if (!checkIsSupportedDeviceOrFinish(this)) {
            return;
        }

        // initialize screen labels and buttons
        initializeViews();

        // battery power setting
        PowerManager powerManager = (PowerManager) getSystemService(Context.POWER_SERVICE);
        mWakeLock = powerManager.newWakeLock(PowerManager.PARTIAL_WAKE_LOCK, "sensors_data_logger:wakelocktag");
        mWakeLock.acquire();

        // Initialize UI state
        mLabelInterfaceTime.setText(R.string.ready_title);
        mStartStopButton.setEnabled(false);
        mStartStopButton.setText("Checking ARCore...");
        
        // Initialize status labels
        mLabelNumberFeatures.setText("N/A");
        mLabelTrackingStatus.setText("INITIALIZING");
        mLabelTrackingFailureReason.setText("N/A");
        mLabelUpdateRate.setText("N/A");
    }


    @Override
    protected void onResume() {
        super.onResume();
        
        if (!hasPermissions(this, REQUIRED_PERMISSIONS)) {
            requestPermissions(REQUIRED_PERMISSIONS, REQUEST_CODE_ANDROID);
            return;
        }

        // Check ARCore availability and request installation if needed
        if (mSession == null) {
            Exception exception = null;
            String message = null;
            try {
                // Check ARCore availability without forcing update
                ArCoreApk.Availability availability = ArCoreApk.getInstance().checkAvailability(this);
                if (availability.isTransient()) {
                    // Re-query at 5Hz while compatibility is checked in the background.
                    new Handler().postDelayed(new Runnable() {
                        @Override
                        public void run() {
                            onResume();
                        }
                    }, 200);
                    return;
                }
                
                if (availability.isSupported()) {
                    // ARCore is supported and available, try to create session directly
                    try {
                        mSession = new Session(/* context= */ this);
                    } catch (UnavailableApkTooOldException e) {
                        // Only request install/update if session creation fails due to old version
                        switch (ArCoreApk.getInstance().requestInstall(this, !mInstallRequested)) {
                            case INSTALL_REQUESTED:
                                mInstallRequested = true;
                                return;
                            case INSTALLED:
                                mSession = new Session(/* context= */ this);
                                break;
                        }
                    } catch (UnavailableArcoreNotInstalledException e) {
                        // ARCore not installed, request installation
                        switch (ArCoreApk.getInstance().requestInstall(this, !mInstallRequested)) {
                            case INSTALL_REQUESTED:
                                mInstallRequested = true;
                                return;
                            case INSTALLED:
                                mSession = new Session(/* context= */ this);
                                break;
                        }
                    }
                } else {
                    // ARCore not supported on this device
                    message = "This device does not support AR";
                    exception = new UnavailableDeviceNotCompatibleException();
                }

            } catch (UnavailableArcoreNotInstalledException
                    | UnavailableUserDeclinedInstallationException e) {
                message = "Please install ARCore from Google Play Store";
                exception = e;
            } catch (UnavailableApkTooOldException e) {
                message = "Please update ARCore from Google Play Store";
                exception = e;
            } catch (UnavailableSdkTooOldException e) {
                message = "Please update this app";
                exception = e;
            } catch (UnavailableDeviceNotCompatibleException e) {
                message = "This device does not support AR";
                exception = e;
            } catch (Exception e) {
                message = "Failed to create AR session: " + e.getMessage();
                exception = e;
            }

            if (message != null) {
                showToast(message);
                Log.e(LOG_TAG, "Exception creating session", exception);
                // Don't return here - allow the app to continue without ARCore
                // but disable recording functionality
                updateUIForNoARCore();
                return;
            }
        }

        // Initialize ARCore session only after successful setup
        if (mARCoreSession == null && mSession != null) {
            try {
                mARCoreSession = new ARCoreSession(this);
                // Initialize ArFragment after ARCore session is confirmed working
                mARCoreSession.initializeArFragment();
                // monitor ARCore information
                displayARCoreInformation();
                // Enable recording button
                enableRecordingButton();
            } catch (Exception e) {
                Log.e(LOG_TAG, "Failed to initialize ARCore session", e);
                showToast("Failed to initialize ARCore session");
                updateUIForNoARCore();
            }
        }
    }


    @Override
    protected void onDestroy() {
        if (mIsRecording.get()) {
            stopRecording();
        }
        if (mSession != null) {
            mSession.close();
            mSession = null;
        }
        if (mWakeLock.isHeld()) {
            mWakeLock.release();
        }
        super.onDestroy();
    }


    // methods
    public void startStopRecording(View view) {
        // Check if ARCore session is available before allowing recording
        if (mARCoreSession == null) {
            showToast("ARCore is not available. Please install ARCore and restart the app.");
            return;
        }
        
        if (!mIsRecording.get()) {

            // start recording sensor measurements when button is pressed
            startRecording();

            // start interface timer on display
            mSecondCounter = 0;
            mInterfaceTimer.schedule(new TimerTask() {
                @Override
                public void run() {
                    mSecondCounter += 1;
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            mLabelInterfaceTime.setText(interfaceIntTime(mSecondCounter));
                        }
                    });
                }
            }, 0, 1000);

        } else {

            // stop recording sensor measurements when button is pressed
            stopRecording();

            // stop interface timer on display
            mInterfaceTimer.cancel();
            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    mLabelInterfaceTime.setText(R.string.ready_title);
                }
            });
        }
    }

    public void detectRFIDTag(View view) {
        // Check if recording is active
        if (!mIsRecording.get()) {
            showToast("Please start recording first");
            return;
        }
        
        // Check if ARCore session is available
        if (mARCoreSession == null) {
            showToast("ARCore session is not available");
            return;
        }
        
        // Trigger RFID tag detection
        mARCoreSession.detectRFIDTag("TAG001");
        showToast("RFID Tag detected!");
    }

    private void startRecording() {

        // output directory for text files
        String outputFolder = null;
        try {
            Log.i(LOG_TAG, "startRecording: Creating output directory manager");
            OutputDirectoryManager folder = new OutputDirectoryManager(this, "", "R_pjinkim_ARCore");
            outputFolder = folder.getOutputDirectory();
            Log.i(LOG_TAG, "startRecording: Output folder created: " + outputFolder);
        } catch (IOException e) {
            Log.e(LOG_TAG, "startRecording: Cannot create output folder.", e);
            showToast("Cannot create output folder");
            e.printStackTrace();
            return;
        }

        // COMPLETE VIO RESET: Destroy and recreate ARCore session for clean start
        try {
            Log.i(LOG_TAG, "startRecording: Starting complete VIO reset...");
            
            // Step 1: Clean up existing ARCore session
            if (mARCoreSession != null) {
                Log.i(LOG_TAG, "startRecording: Cleaning up existing ARCore session");
                mARCoreSession.cleanupSession();
                mARCoreSession = null;
            }
            
            // Step 2: Reset ARCore Session at native level
            if (mSession != null) {
                Log.i(LOG_TAG, "startRecording: Closing native ARCore session");
                mSession.close();
                mSession = null;
            }
            
            // Step 3: Brief pause to ensure complete cleanup
            Thread.sleep(100);
            
            // Step 4: Create fresh ARCore session
            Log.i(LOG_TAG, "startRecording: Creating fresh ARCore session");
            mSession = new com.google.ar.core.Session(this);
            
            // Step 5: Create new ARCoreSession wrapper
            Log.i(LOG_TAG, "startRecording: Creating new ARCoreSession wrapper");
            mARCoreSession = new ARCoreSession(this);
            
            // Step 6: Initialize ArFragment with clean state
            Log.i(LOG_TAG, "startRecording: Initializing fresh ArFragment");
            mARCoreSession.initializeArFragment();
            
            // Step 7: Brief stabilization period
            Thread.sleep(200);
            
            Log.i(LOG_TAG, "startRecording: VIO reset completed successfully");
            
        } catch (Exception e) {
            Log.e(LOG_TAG, "startRecording: Failed to reset VIO sensors", e);
            showToast("Failed to reset VIO sensors: " + e.getMessage());
            return;
        }

        // Start fresh session with clean VIO state
        Log.i(LOG_TAG, "startRecording: Starting fresh ARCore session with folder: " + outputFolder);
        mARCoreSession.startSession(outputFolder);
        mIsRecording.set(true);
        Log.i(LOG_TAG, "startRecording: Recording state set to true with clean VIO");

        // update Start/Stop button UI
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                mStartStopButton.setEnabled(true);
                mStartStopButton.setText(R.string.stop_title);
                mDetectTagButton.setEnabled(true);
            }
        });
        showToast("Recording starts with fresh VIO!");
    }


    protected void stopRecording() {
        mHandler.post(new Runnable() {
            @Override
            public void run() {

                // stop ARCore session only if it exists
                if (mARCoreSession != null) {
                    mARCoreSession.stopSession();
                }
                mIsRecording.set(false);

                // Copy files to external storage for easy access
                copyFilesToExternalStorage();

                // update screen UI and button
                showToast("Recording stops! Files saved to Downloads folder.");
                resetUI();
            }
        });
    }

    private void copyFilesToExternalStorage() {
        try {
            // Get the most recent recording folder
            File internalDir = getFilesDir();
            File[] folders = internalDir.listFiles();
            if (folders == null || folders.length == 0) {
                Log.w(LOG_TAG, "No recording folders found");
                return;
            }

            // Find the most recent folder
            File mostRecentFolder = null;
            long lastModified = 0;
            for (File folder : folders) {
                if (folder.isDirectory() && folder.lastModified() > lastModified) {
                    lastModified = folder.lastModified();
                    mostRecentFolder = folder;
                }
            }

            if (mostRecentFolder == null) {
                Log.w(LOG_TAG, "No recent folder found");
                return;
            }

            // Create destination folder in Downloads
            File downloadsDir = new File(getExternalFilesDir(null), "ARCore_Logs");
            if (!downloadsDir.exists()) {
                downloadsDir.mkdirs();
            }

            File destFolder = new File(downloadsDir, mostRecentFolder.getName());
            if (!destFolder.exists()) {
                destFolder.mkdirs();
            }

            // Copy files
            File[] files = mostRecentFolder.listFiles();
            if (files != null) {
                for (File file : files) {
                    if (file.isFile()) {
                        copyFile(file, new File(destFolder, file.getName()));
                        Log.i(LOG_TAG, "Copied file: " + file.getName() + " to " + destFolder.getAbsolutePath());
                    }
                }
            }

            Log.i(LOG_TAG, "Files copied to: " + destFolder.getAbsolutePath());
            
        } catch (Exception e) {
            Log.e(LOG_TAG, "Error copying files to external storage", e);
        }
    }

    private void copyFile(File source, File dest) throws IOException {
        java.io.FileInputStream fis = new java.io.FileInputStream(source);
        java.io.FileOutputStream fos = new java.io.FileOutputStream(dest);
        
        byte[] buffer = new byte[1024];
        int length;
        while ((length = fis.read(buffer)) > 0) {
            fos.write(buffer, 0, length);
        }
        
        fis.close();
        fos.close();
    }

    public void showToast(final String text) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                Toast.makeText(MainActivity.this, text, Toast.LENGTH_SHORT).show();
            }
        });
    }


    private void resetUI() {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                mLabelNumberFeatures.setText("N/A");
                mLabelTrackingStatus.setText("N/A");
                mLabelTrackingFailureReason.setText("N/A");
                mLabelUpdateRate.setText("N/A");

                mStartStopButton.setEnabled(true);
                mStartStopButton.setText(R.string.start_title);
                mDetectTagButton.setEnabled(false);
            }
        });
    }


    public static boolean checkIsSupportedDeviceOrFinish(final Activity activity) {

        // check Android version
        if (Build.VERSION.SDK_INT < Build.VERSION_CODES.N) {
            Log.e(LOG_TAG, "Sceneform requires Android N or later");
            Toast.makeText(activity, "Sceneform requires Android N or later", Toast.LENGTH_LONG).show();
            activity.finish();
            return false;
        }

        // get current OpenGL version
        String openGlVersionString = ((ActivityManager) activity.getSystemService(Context.ACTIVITY_SERVICE))
                .getDeviceConfigurationInfo()
                .getGlEsVersion();

        // check OpenGL version
        if (Double.parseDouble(openGlVersionString) < MIN_OPENGL_VERSION) {
            Log.e(LOG_TAG, "Sceneform requires OpenGL ES 3.0 later");
            Toast.makeText(activity, "Sceneform requires OpenGL ES 3.0 or later", Toast.LENGTH_LONG).show();
            activity.finish();
            return false;
        }
        return true;
    }


    @Override
    public void onBackPressed() {

        // nullify back button when recording starts
        if (!mIsRecording.get()) {
            super.onBackPressed();
        }
    }


    @Override
    public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        
        if (requestCode == REQUEST_CODE_ANDROID) {
            if (hasPermissions(this, REQUIRED_PERMISSIONS)) {
                // Permissions granted, resume ARCore initialization
                Log.d(LOG_TAG, "Permissions granted, resuming ARCore initialization");
                // Call onResume logic again to continue with ARCore setup
                if (mSession == null) {
                    // Restart the ARCore initialization process
                    onResume();
                }
            } else {
                // Permissions denied
                showToast("Camera permission is required for ARCore functionality");
                Log.e(LOG_TAG, "Required permissions not granted");
                finish();
            }
        }
    }


    private static boolean hasPermissions(Context context, String... permissions) {

        // check Android hardware permissions
        for (String permission : permissions) {
            if (ContextCompat.checkSelfPermission(context, permission) != PackageManager.PERMISSION_GRANTED) {
                return false;
            }
        }
        return true;
    }


    private String interfaceIntTime(final int second) {

        // check second input
        if (second < 0) {
            Log.e(LOG_TAG, "interfaceIntTime: Second cannot be negative.");
            return null;
        }

        // extract hour, minute, second information from second
        int input = second;
        int hours = input / 3600;
        input = input % 3600;
        int mins = input / 60;
        int secs = input % 60;

        // return interface int time
        return String.format(Locale.US, "%02d:%02d:%02d", hours, mins, secs);
    }


    private void initializeViews() {

        mLabelNumberFeatures = (TextView) findViewById(R.id.label_number_features);
        mLabelTrackingStatus = (TextView) findViewById(R.id.label_tracking_status);
        mLabelTrackingFailureReason = (TextView) findViewById(R.id.label_tracking_failure_reason);
        mLabelUpdateRate = (TextView) findViewById(R.id.label_update_rate);

        mStartStopButton = (Button) findViewById(R.id.button_start_stop);
        mDetectTagButton = (Button) findViewById(R.id.button_detect_tag);
        mLabelInterfaceTime = (TextView) findViewById(R.id.label_interface_time);
    }


    private void displayARCoreInformation() {

        // get ARCore tracking information
        int numberOfFeatures = mARCoreSession.getNumberOfFeatures();
        TrackingState trackingState = mARCoreSession.getTrackingState();
        TrackingFailureReason trackingFailureReason =  mARCoreSession.getTrackingFailureReason();
        double updateRate = mARCoreSession.getUpdateRate();

        // update current screen (activity)
        runOnUiThread(new Runnable() {
            @Override
            public void run() {

                // determine TrackingState text
                String ARCoreTrackingState = "";
                if (trackingState == TrackingState.PAUSED) {
                    ARCoreTrackingState = "PAUSED";
                } else if (trackingState == TrackingState.STOPPED) {
                    ARCoreTrackingState = "STOPPED";
                } else if (trackingState == TrackingState.TRACKING) {
                    ARCoreTrackingState = "TRACKING";
                } else {
                    ARCoreTrackingState = "ERROR?";
                }

                // determine TrackingFailureReason text
                String ARCoreTrackingFailureReason = "";
                if (trackingFailureReason == TrackingFailureReason.BAD_STATE) {
                    ARCoreTrackingFailureReason = "BAD STATE";
                } else if (trackingFailureReason == TrackingFailureReason.EXCESSIVE_MOTION) {
                    ARCoreTrackingFailureReason = "FAST MOTION";
                } else if (trackingFailureReason == TrackingFailureReason.INSUFFICIENT_FEATURES) {
                    ARCoreTrackingFailureReason = "LOW FEATURES";
                } else if (trackingFailureReason == TrackingFailureReason.INSUFFICIENT_LIGHT) {
                    ARCoreTrackingFailureReason = "LOW LIGHT";
                } else if (trackingFailureReason == TrackingFailureReason.NONE) {
                    ARCoreTrackingFailureReason = "NONE";
                } else {
                    ARCoreTrackingFailureReason = "ERROR?";
                }

                // update interface screen labels
                mLabelNumberFeatures.setText(String.format(Locale.US, "%05d", numberOfFeatures));
                mLabelTrackingStatus.setText(ARCoreTrackingState);
                mLabelTrackingFailureReason.setText(ARCoreTrackingFailureReason);
                mLabelUpdateRate.setText(String.format(Locale.US, "%.3f Hz", updateRate));
            }
        });

        // determine display update rate (100 ms)
        final long displayInterval = 100;
        mHandler.postDelayed(new Runnable() {
            @Override
            public void run() {
                displayARCoreInformation();
            }
        }, displayInterval);
    }

    private void updateUIForNoARCore() {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                mLabelNumberFeatures.setText("N/A");
                mLabelTrackingStatus.setText("NO ARCORE");
                mLabelTrackingFailureReason.setText("NOT INSTALLED");
                mLabelUpdateRate.setText("N/A");
                
                // Disable the start/stop button
                mStartStopButton.setEnabled(false);
                mStartStopButton.setText("ARCore Required");
                mDetectTagButton.setEnabled(false);
            }
        });
    }

    private void enableRecordingButton() {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                mStartStopButton.setEnabled(true);
                mStartStopButton.setText(R.string.start_title);
                mDetectTagButton.setEnabled(true);
            }
        });
    }
}
