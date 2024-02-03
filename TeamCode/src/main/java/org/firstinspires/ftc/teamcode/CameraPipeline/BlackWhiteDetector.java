package org.firstinspires.ftc.teamcode.CameraPipeline;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;
import java.util.List;

public class BlackWhiteDetector extends OpenCvPipeline {

    static int color_zone = 3; // Default to 3

    int toggleShow = 1;

    Mat original;

    Mat zone1;
    Mat zone2;

    Scalar avgColor1;
    Scalar avgColor2;

    double distance1 = 1;
    double distance2 = 1;

    double max_distance = 0;

    @Override
    public Mat processFrame(Mat input) {
        // Creating duplicate of original frame with no edits
        original = input.clone();

        // Defining Zones
        // Rect(top left x, top left y, bottom right x, bottom right y)
        zone1 = input.submat(new Rect(75, 380, 300, 230));
        zone2 = input.submat(new Rect(735, 380, 300, 230));

        // Convert input frame to HSV color space
        Imgproc.cvtColor(zone1, zone1, Imgproc.COLOR_RGB2HSV_FULL);
        Imgproc.cvtColor(zone2, zone2, Imgproc.COLOR_RGB2HSV_FULL);

        // Continue with your color detection logic...
        // For example, you might calculate the average color in HSV space as follows:

        avgColor1 = Core.mean(zone1);
        avgColor2 = Core.mean(zone2);

        zone1.setTo(avgColor1);
        zone2.setTo(avgColor2);

        // Check if average colors are in the range from white to black
        boolean isZone1WhiteToBlack = isInRange(avgColor1, Arrays.asList(new Scalar(0, 0, 0), new Scalar(180, 255, 30)));
        boolean isZone2WhiteToBlack = isInRange(avgColor2, Arrays.asList(new Scalar(0, 0, 0), new Scalar(180, 255, 30)));

        if (isZone1WhiteToBlack && isZone2WhiteToBlack) {
            max_distance =  -1;
        } else {
            if (isZone1WhiteToBlack) {
                color_zone = 1;
            } else if (isZone2WhiteToBlack) {
                color_zone = 2;
            }
        }

        // Allowing for the showing of the averages on the stream
        if (toggleShow == 1) {
            return input;
        } else {
            return original;
        }
    }

    public double color_distance(Scalar color1, List<Scalar> colorRange) {
        double h1 = color1.val[0];
        double s1 = color1.val[1];
        double v1 = color1.val[2];

        double hMin = colorRange.get(0).val[0];
        double sMin = colorRange.get(0).val[1];
        double vMin = colorRange.get(0).val[2];

        double hMax = colorRange.get(1).val[0];
        double sMax = colorRange.get(1).val[1];
        double vMax = colorRange.get(1).val[2];

        double distance = Math.sqrt(Math.pow(Math.max(0, Math.max(h1 - hMax, hMin - h1)), 2)
                + Math.pow(Math.max(0, Math.max(s1 - sMax, sMin - s1)), 2)
                + Math.pow(Math.max(0, Math.max(v1 - vMax, vMin - v1)), 2));

        return distance;
    }

    //check if the zone is in range
    public boolean isInRange(Scalar color, List<Scalar> colorRange) {
        return (color_distance(color, colorRange) >= 0 && color_distance(color, colorRange) <= 100);
    }

    public int get_element_zone() {
        return color_zone;
    }
}
