package com.pjinkim.arcore_data_logger;

import com.google.ar.sceneform.math.Vector3;

import java.util.ArrayList;
import java.util.HashMap;

public class AccumulatedPointCloud {

    // properties
    private static final int MAX_POINTS = 50000; // Reduced from 100000 to prevent memory issues
    private ArrayList<Vector3> mPoints = new ArrayList<Vector3>();
    private ArrayList<Vector3> mColors = new ArrayList<Vector3>();
    private HashMap<Integer, Integer> mIdentifiedIndices = new HashMap<Integer, Integer>();
    private int mNumberOfFeatures = 0;


    // constructor
    public AccumulatedPointCloud() {
        // No initialization needed for HashMap
    }


    // methods
    public void appendPointCloud(int pointID, float pointX, float pointY, float pointZ, float r, float g, float b) {
        if (mIdentifiedIndices.containsKey(pointID)) {
            int existingIndex = mIdentifiedIndices.get(pointID);
            Vector3 pointPosition = new Vector3(pointX, pointY, pointZ);
            Vector3 pointColor = new Vector3(r, g, b);
            mPoints.set(existingIndex, pointPosition);
            mColors.set(existingIndex, pointColor);
        } else {
            // Check if we've reached the maximum number of points
            if (mNumberOfFeatures >= MAX_POINTS) {
                // Skip adding new points if we've reached the limit
                return;
            }
            
            mIdentifiedIndices.put(pointID, mNumberOfFeatures);
            Vector3 pointPosition = new Vector3(pointX, pointY, pointZ);
            Vector3 pointColor = new Vector3(r, g, b);
            mPoints.add(pointPosition);
            mColors.add(pointColor);
            mNumberOfFeatures++;
        }
    }


    // getter and setter
    public int getNumberOfFeatures() {
        return mNumberOfFeatures;
    }

    public ArrayList<Vector3> getPoints() {
        return mPoints;
    }

    public ArrayList<Vector3> getColors() {
        return mColors;
    }
}
