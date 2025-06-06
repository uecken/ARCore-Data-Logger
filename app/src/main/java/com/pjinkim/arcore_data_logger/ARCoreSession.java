package com.pjinkim.arcore_data_logger;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Color;
import android.graphics.ImageFormat;
import android.graphics.Matrix;
import android.graphics.Rect;
import android.graphics.YuvImage;
import android.media.Image;
import android.util.Log;

import androidx.annotation.NonNull;

import com.google.ar.core.Camera;
import com.google.ar.core.Frame;
import com.google.ar.core.PointCloud;
import com.google.ar.core.Pose;
import com.google.ar.core.TrackingFailureReason;
import com.google.ar.core.TrackingState;
import com.google.ar.core.exceptions.NotYetAvailableException;
import com.google.ar.sceneform.FrameTime;
import com.google.ar.sceneform.math.Vector3;
import com.google.ar.sceneform.ux.ArFragment;

import java.io.BufferedWriter;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.security.KeyException;
import java.util.ArrayList;
import java.util.Locale;
import java.util.concurrent.atomic.AtomicBoolean;

public class ARCoreSession {

    // properties
    private static final String LOG_TAG = ARCoreSession.class.getName();
    private static final long mulSecondToNanoSecond = 1000000000;
    private long previousTimestamp = 0;

    private MainActivity mContext;
    private ArFragment mArFragment;
    private PointCloudNode mPointCloudNode;
    private AccumulatedPointCloud mAccumulatedPointCloud;
    private WorldToScreenTranslator mWorldToScreenTranslator;
    private ARCoreResultStreamer mFileStreamer = null;

    private AtomicBoolean mIsRecording = new AtomicBoolean(false);
    private AtomicBoolean mIsWritingFile = new AtomicBoolean(false);

    private int mNumberOfFeatures = 0;
    private TrackingState mTrackingState;
    private TrackingFailureReason mTrackingFailureReason;
    private double mUpdateRate = 0;


    // constructor
    public ARCoreSession(@NonNull MainActivity context) {

        // initialize object
        mContext = context;
        mAccumulatedPointCloud = new AccumulatedPointCloud();
        mWorldToScreenTranslator = new WorldToScreenTranslator();
        
        // ArFragment will be initialized later after ARCore session is confirmed
    }

    // Initialize ArFragment after ARCore session is confirmed working
    public void initializeArFragment() {
        if (mArFragment == null) {
            // Create ArFragment dynamically
            mArFragment = new ArFragment();
            
            // Add the fragment to the container
            mContext.getSupportFragmentManager()
                    .beginTransaction()
                    .replace(R.id.ux_fragment, mArFragment)
                    .commit();
            
            // Wait for fragment to be ready, then configure it
            mContext.getSupportFragmentManager().executePendingTransactions();
            
            // Configure the ArFragment
            mArFragment.getArSceneView().getPlaneRenderer().setVisible(false);
            mArFragment.getArSceneView().getScene().addOnUpdateListener(this::onUpdateFrame);

            // render 3D point cloud on the screen
            mPointCloudNode = new PointCloudNode(mContext);
            mArFragment.getArSceneView().getScene().addChild(mPointCloudNode);
        }
    }


    // methods
    public void startSession(String streamFolder) {

        // initialize text file stream
        if (streamFolder != null) {
            try {
                Log.i(LOG_TAG, "startSession: Initializing file streamer with folder: " + streamFolder);
                mFileStreamer = new ARCoreResultStreamer(mContext, streamFolder);
                mIsWritingFile.set(true);
                Log.i(LOG_TAG, "startSession: File streamer initialized successfully");
            } catch (IOException e) {
                Log.e(LOG_TAG, "startSession: Cannot create file for ARCore tracking results.", e);
                mContext.showToast("Cannot create file for ARCore tracking results.");
                e.printStackTrace();
            }
        } else {
            Log.w(LOG_TAG, "startSession: streamFolder is null, no files will be created");
        }
        mIsRecording.set(true);
        Log.i(LOG_TAG, "startSession: Recording started");
    }


    public void stopSession() {

        // save ARCore 3D point cloud only for visualization
        ArrayList<Vector3> pointsPosition = mAccumulatedPointCloud.getPoints();
        ArrayList<Vector3> pointsColor = mAccumulatedPointCloud.getColors();
        for (int i = 0; i < pointsPosition.size(); i++) {
            Vector3 currentPointPosition = pointsPosition.get(i);
            Vector3 currentPointColor = pointsColor.get(i);
            float pointX = currentPointPosition.x;
            float pointY = currentPointPosition.y;
            float pointZ = currentPointPosition.z;
            float r = currentPointColor.x;
            float g = currentPointColor.y;
            float b = currentPointColor.z;
            try {
                mFileStreamer.addARCorePointRecord(pointX, pointY, pointZ, r, g, b);
            } catch (IOException | KeyException e) {
                Log.d(LOG_TAG, "stopSession: Something is wrong.");
                e.printStackTrace();
            }
        }

        // close text file and reset variables
        if (mIsWritingFile.get()) {
            try {
                mFileStreamer.endFiles();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        mIsWritingFile.set(false);
        mIsRecording.set(false);
    }

    /**
     * Clean up ARCore session resources
     * This method is called when resetting the VIO system for a fresh start
     */
    public void cleanupSession() {
        Log.i(LOG_TAG, "cleanupSession: Cleaning up ARCore session resources");
        
        try {
            // Stop any ongoing recording
            if (mIsRecording.get()) {
                stopSession();
            }
            
            // Clean up file streamer
            if (mFileStreamer != null && mIsWritingFile.get()) {
                try {
                    mFileStreamer.endFiles();
                } catch (IOException e) {
                    Log.w(LOG_TAG, "cleanupSession: Error closing file streamer", e);
                }
                mFileStreamer = null;
            }
            
            // Reset state variables
            mIsRecording.set(false);
            mIsWritingFile.set(false);
            mNumberOfFeatures = 0;
            mTrackingState = null;
            mTrackingFailureReason = null;
            mUpdateRate = 0;
            previousTimestamp = 0;
            
            // Clear accumulated point cloud
            if (mAccumulatedPointCloud != null) {
                mAccumulatedPointCloud = new AccumulatedPointCloud();
            }
            
            // Remove ArFragment from UI if it exists
            if (mArFragment != null && mContext != null) {
                try {
                    mContext.getSupportFragmentManager()
                            .beginTransaction()
                            .remove(mArFragment)
                            .commitAllowingStateLoss();
                    mArFragment = null;
                } catch (Exception e) {
                    Log.w(LOG_TAG, "cleanupSession: Error removing ArFragment", e);
                    mArFragment = null;
                }
            }
            
            // Clear point cloud node
            mPointCloudNode = null;
            
            Log.i(LOG_TAG, "cleanupSession: Cleanup completed successfully");
            
        } catch (Exception e) {
            Log.e(LOG_TAG, "cleanupSession: Error during cleanup", e);
        }
    }

    private void onUpdateFrame(FrameTime frameTime) {

        // Check if ArFragment is initialized
        if (mArFragment == null) {
            return;
        }

        // set some variables
        boolean isFileSaved = (mIsRecording.get() && mIsWritingFile.get());

        // obtain current ARCore information
        mArFragment.onUpdate(frameTime);
        Frame frame = mArFragment.getArSceneView().getArFrame();
        
        // Check if frame is available
        if (frame == null) {
            return;
        }
        
        Camera camera = frame.getCamera();

        // update ARCore measurements
        long timestamp = frame.getTimestamp();
        double updateRate = (double) mulSecondToNanoSecond / (double) (timestamp - previousTimestamp);
        previousTimestamp = timestamp;

        TrackingState trackingState = camera.getTrackingState();
        TrackingFailureReason trackingFailureReason = camera.getTrackingFailureReason();
        Pose T_gc = frame.getAndroidSensorPose();

        float qx = T_gc.qx();
        float qy = T_gc.qy();
        float qz = T_gc.qz();
        float qw = T_gc.qw();

        float tx = T_gc.tx();
        float ty = T_gc.ty();
        float tz = T_gc.tz();

        // update 3D point cloud from ARCore
        PointCloud pointCloud = frame.acquirePointCloud();
        IntBuffer bufferPointID = pointCloud.getIds();
        FloatBuffer bufferPoint3D = pointCloud.getPoints();
        mPointCloudNode.visualize(pointCloud);
        int numberOfFeatures = mAccumulatedPointCloud.getNumberOfFeatures();
        pointCloud.release();

        // display and save ARCore information
        try {
            mNumberOfFeatures = numberOfFeatures;
            mTrackingState = trackingState;
            mTrackingFailureReason = trackingFailureReason;
            mUpdateRate = updateRate;
            if (isFileSaved) {

                // 1) record ARCore 6-DoF sensor pose
                mFileStreamer.addARCorePoseRecord(timestamp, qx, qy, qz, qw, tx, ty, tz);

                // 2) record ARCore 3D point cloud only for visualization
                Image imageFrame = frame.acquireCameraImage();
                Bitmap imageBitmap = imageToBitmap(imageFrame);
                imageFrame.close();
                for (int i = 0; i < (bufferPoint3D.limit() / 4); i++) {

                    // check each point's confidence level
                    float pointConfidence = bufferPoint3D.get(i * 4 + 3);
                    if (pointConfidence < 0.5) {
                        continue;
                    }

                    // obtain point ID and XYZ world position
                    int pointID = bufferPointID.get(i);
                    float pointX = bufferPoint3D.get(i * 4);
                    float pointY = bufferPoint3D.get(i * 4 + 1);
                    float pointZ = bufferPoint3D.get(i * 4 + 2);

                    // get each point RGB color information
                    float[] worldPosition = new float[]{pointX, pointY, pointZ};
                    Vector3 pointColor = getScreenPixel(worldPosition, imageBitmap);
                    if (pointColor == null) {
                        continue;
                    }

                    // append each point position and color information
                    mAccumulatedPointCloud.appendPointCloud(pointID, pointX, pointY, pointZ, pointColor.x, pointColor.y, pointColor.z);
                }
            }
        } catch (IOException | KeyException | NotYetAvailableException e) {
            Log.d(LOG_TAG, "onUpdateFrame: Something is wrong.");
            e.printStackTrace();
        }
    }


    private Bitmap imageToBitmap (Image image) {
        int width = image.getWidth();
        int height = image.getHeight();

        byte[] nv21;
        ByteBuffer yBuffer = image.getPlanes()[0].getBuffer();
        ByteBuffer uBuffer = image.getPlanes()[1].getBuffer();
        ByteBuffer vBuffer = image.getPlanes()[2].getBuffer();

        int ySize = yBuffer.remaining();
        int uSize = uBuffer.remaining();
        int vSize = vBuffer.remaining();

        nv21 = new byte[ySize + uSize + vSize];

        // U and V are swapped
        yBuffer.get(nv21, 0, ySize);
        vBuffer.get(nv21, ySize, vSize);
        uBuffer.get(nv21, ySize + vSize, uSize);

        YuvImage yuvImage = new YuvImage(nv21, ImageFormat.NV21, width, height, null);
        ByteArrayOutputStream os = new ByteArrayOutputStream();
        yuvImage.compressToJpeg(new Rect(0, 0, width, height), 100, os);
        byte[] jpegByteArray = os.toByteArray();
        Bitmap bitmap = BitmapFactory.decodeByteArray(jpegByteArray, 0, jpegByteArray.length);

        Matrix matrix = new Matrix();
        matrix.setRotate(90);

        return Bitmap.createBitmap(bitmap, 0, 0, bitmap.getWidth(), bitmap.getHeight(), matrix, true);
    }


    private Vector3 getScreenPixel(float[] worldPosition, Bitmap imageBitmap) throws NotYetAvailableException {

        // clip to screen space (ViewMatrix * ProjectionMatrix * Anchor Matrix)
        double[] pos2D = mWorldToScreenTranslator.worldToScreen(imageBitmap.getWidth(), imageBitmap.getHeight(), mArFragment.getArSceneView().getArFrame().getCamera(), worldPosition);

        // check if inside the screen
        if ((pos2D[0] < 0) || (pos2D[0] > imageBitmap.getWidth()) || (pos2D[1] < 0) || (pos2D[1] > imageBitmap.getHeight())) {
            return null;
        }

        int pixel = imageBitmap.getPixel((int) pos2D[0], (int) pos2D[1]);
        int r = Color.red(pixel);
        int g = Color.green(pixel);
        int b = Color.blue(pixel);
        Vector3 pointColor = new Vector3(r, g, b);

        return pointColor;
    }


    /**
     * Detect a single RFID tag (convenience method)
     * This method wraps the multiple tags detection for single tag use
     * @param tagId Single tag ID to detect
     */
    public void detectRFIDTag(String tagId) {
        if (tagId == null || tagId.trim().isEmpty()) {
            Log.w(LOG_TAG, "detectRFIDTag: Invalid tag ID provided");
            return;
        }
        
        // Use the multiple tags method with a single tag
        String[] singleTagArray = {tagId};
        detectMultipleRFIDTags(singleTagArray);
        
        Log.i(LOG_TAG, "detectRFIDTag: Single tag detected: " + tagId);
    }

    /**
     * Detect multiple RFID tags simultaneously with the same timestamp and pose
     * This is more efficient for simultaneous detections as it uses the same frame data
     * @param tagIds Array of tag IDs detected simultaneously
     */
    public void detectMultipleRFIDTags(String[] tagIds) {
        if (!mIsRecording.get() || !mIsWritingFile.get()) {
            Log.w(LOG_TAG, "detectMultipleRFIDTags: Not recording, ignoring tag detections");
            return;
        }

        if (mArFragment == null) {
            Log.w(LOG_TAG, "detectMultipleRFIDTags: ArFragment not initialized");
            return;
        }

        if (tagIds == null || tagIds.length == 0) {
            Log.w(LOG_TAG, "detectMultipleRFIDTags: No tag IDs provided");
            return;
        }

        try {
            Frame frame = mArFragment.getArSceneView().getArFrame();
            if (frame == null) {
                Log.w(LOG_TAG, "detectMultipleRFIDTags: No current frame available");
                return;
            }

            // Get current pose data (same for all simultaneous detections)
            long timestamp = frame.getTimestamp();
            Camera camera = frame.getCamera();
            Pose cameraPose = camera.getPose();
            
            // Extract quaternion and translation from camera pose
            float[] quaternion = cameraPose.getRotationQuaternion();
            float[] translation = cameraPose.getTranslation();

            // Record pose with all tag IDs in a single line
            mFileStreamer.addARCorePoseWithMultipleTagsRecord(timestamp, 
                quaternion[0], quaternion[1], quaternion[2], quaternion[3],
                translation[0], translation[1], translation[2], tagIds);
            
            Log.i(LOG_TAG, "detectMultipleRFIDTags: Recorded " + tagIds.length + 
                  " tags simultaneously at pose (" + translation[0] + ", " + 
                  translation[1] + ", " + translation[2] + ")");

        } catch (IOException | KeyException e) {
            Log.e(LOG_TAG, "detectMultipleRFIDTags: Error recording tag detections", e);
        }
    }


    // definition of 'ARCoreResultStreamer' class
    class ARCoreResultStreamer extends FileStreamer {

        // properties
        private BufferedWriter mWriterPose;
        private BufferedWriter mWriterPoint;


        // constructor
        ARCoreResultStreamer(final Context context, final String outputFolder) throws IOException {
            super(context, outputFolder);
            addFile("ARCore_sensor_pose", "ARCore_sensor_pose.txt");
            addFile("ARCore_point_cloud", "ARCore_point_cloud.txt");
            mWriterPose = getFileWriter("ARCore_sensor_pose");
            mWriterPoint = getFileWriter("ARCore_point_cloud");
        }


        // methods
        public void addARCorePoseRecord(long timestamp, float qx, float qy, float qz, float qw, float tx, float ty, float tz) throws IOException, KeyException {

            // execute the block with only one thread
            synchronized (this) {

                // record timestamp and 6-DoF device pose in text file
                StringBuilder stringBuilder = new StringBuilder();
                stringBuilder.append(timestamp);
                stringBuilder.append(String.format(Locale.US, " %.6f %.6f %.6f %.6f %.6f %.6f %.6f", qx, qy, qz, qw, tx, ty, tz));
                stringBuilder.append(" \n");
                mWriterPose.write(stringBuilder.toString());
            }
        }


        public void addARCorePoseWithMultipleTagsRecord(long timestamp, float qx, float qy, float qz, float qw, float tx, float ty, float tz, String[] tagIds) throws IOException, KeyException {

            // execute the block with only one thread
            synchronized (this) {

                // record timestamp, 6-DoF device pose, and all tag IDs in a single line
                StringBuilder stringBuilder = new StringBuilder();
                stringBuilder.append(timestamp);
                stringBuilder.append(String.format(Locale.US, " %.6f %.6f %.6f %.6f %.6f %.6f %.6f", qx, qy, qz, qw, tx, ty, tz));
                
                // Add all tag IDs to the same line
                for (String tagId : tagIds) {
                    stringBuilder.append(" " + tagId);
                }
                
                stringBuilder.append(" \n");
                mWriterPose.write(stringBuilder.toString());
            }
        }


        public void addARCorePointRecord(final float pointX, final float pointY, final float pointZ, final float r, final float g, final float b) throws IOException, KeyException {

            // execute the block with only one thread
            synchronized (this) {

                // record 3D point cloud in text file
                StringBuilder stringBuilder = new StringBuilder();
                stringBuilder.append(String.format(Locale.US, "%.6f %.6f %.6f %.2f %.2f %.2f", pointX, pointY, pointZ, r, g, b));
                stringBuilder.append(" \n");
                mWriterPoint.write(stringBuilder.toString());
            }
        }


        @Override
        public void endFiles() throws IOException {

            // execute the block with only one thread
            synchronized (this) {
                mWriterPose.flush();
                mWriterPose.close();
                mWriterPoint.flush();
                mWriterPoint.close();
            }
        }
    }


    // getter and setter
    public int getNumberOfFeatures() {
        return mNumberOfFeatures;
    }

    public TrackingState getTrackingState() {
        return mTrackingState;
    }

    public TrackingFailureReason getTrackingFailureReason() {
        return mTrackingFailureReason;
    }

    public double getUpdateRate() {
        return mUpdateRate;
    }
}
