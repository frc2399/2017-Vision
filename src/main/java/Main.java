import java.util.ArrayList;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
//import org.w3c.dom.css.Rect;

import edu.wpi.cscore.*;
import edu.wpi.first.wpilibj.networktables.*;
import edu.wpi.first.wpilibj.tables.*;

public class Main {
	public static void main(String[] args) {
		// Loads our OpenCV library. This MUST be included
		System.loadLibrary("opencv_java310");

		// Connect NetworkTables, and get access to the publishing table
		NetworkTable.setClientMode();
		// Set your team number here
		NetworkTable.setTeam(2399);

		NetworkTable.initialize();

		NetworkTable smartDash = NetworkTable.getTable("SmartDashboard");

		// This is the network port you want to stream the raw received image to
		// By rules, this has to be between 1180 and 1190, so 1185 is a good
		// choice
		int streamPort = 1185;

		// setting up constants for vision calculation
		int cameraFieldOfView = 60;
		// double imageWidth = 320;

		// This stores our reference to our mjpeg server for streaming the input
		// image
		MjpegServer inputStream = new MjpegServer("MJPEG Server", streamPort);

		UsbCamera camera = setUsbCamera(0, inputStream);
		// Set the resolution for our camera, since this is over USB
		camera.setResolution(320, 240);

		// This creates a CvSink for us to use. This grabs images from our
		// selected camera,
		// and will allow us to use those images in opencv
		CvSink imageSink = new CvSink("CV Image Grabber");
		imageSink.setSource(camera);

		// This creates a CvSource to use. This will take in a Mat image that
		// has had OpenCV operations
		// operations
		CvSource imageSource = new CvSource("CV Image Source", VideoMode.PixelFormat.kMJPEG, 320, 240, 30);
		MjpegServer cvStream = new MjpegServer("CV Image Stream", 1186);
		cvStream.setSource(imageSource);

		// All Mats and Lists should be stored outside the loop to avoid
		// allocations
		// as they are expensive to create
		Mat inputImage = new Mat();

		Mat rgb = new Mat();

		double[] rgbThresholdRed = { 0, 244 };
		double[] rgbThresholdGreen = { 255, 255 };
		double[] rgbThresholdBlue = { 133, 255 };

		double centerX = 0.0;
		double height = 0.0;
		double width = 0.0;
		// Point tRight = new Point (0, 0);
		// Point bLeft = new Point(0.0);
		double u;
		double v;
		double fieldOfView = 60;
		double heightOfCameraOnRobot = 37;
		// TODO make idouble boilerHieght equivilant to actual height of boiler
		// when finished
		double heightOfBoilerTape = 39.5; // this is from the top of the 2 in
											// boiler retroreflective tape to
											// the floor and in inches
		double boilerHeightMinusCameraHeight = heightOfBoilerTape - heightOfCameraOnRobot;
		double distanceToTarget;
		double xCenterOfImage;
		double yCenterOfImage;
		double fovToWidthRatio = 60 / 320;
		double halfOfContourLength;
		double degreesToTarget;
		double imageHeight = 240;
		double imageWidth = 320;
		double xOverZ;
		double yOverZ;
		double horizontalAngleToTarget;
		double verticalAngleToTarget;
		double focalLength = 324.571;
		double minArea = 100.0;
		double maxArea = 200.0;
		double minPerimeter = 275.0;
		double minWidth = 55.0;
		double maxWidth = 67.0;
		double minHeight = 0.0;
		double maxHeight = 1000.0;
		double[] solidity = { 0, 100.0 };
		double maxVertices = 1000000.0;
		double minVertices = 0.0;
		double minRatio = 0.0;
		double maxRatio = 0.2;
		ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();
		Point anchor = new Point(-1, 1);
		double times = 1.0;
		int borderType = Core.BORDER_CONSTANT;
		Scalar borderValue = new Scalar(-1);
		xCenterOfImage = ((imageWidth / 2) - 0.5);
		yCenterOfImage = ((imageHeight / 2) - 0.5);
		double actualWidth = 22.75;
		double centerOfImage = imageWidth / 2;
		// Infinitely process image
		while (true) {
			// Grab a frame. If it has a frame time of 0, there was an error.
			// Just skip and continue
			long frameTime = imageSink.grabFrame(inputImage);
			if (frameTime == 0)
				continue;

			Imgproc.cvtColor(inputImage, rgb, Imgproc.COLOR_BGR2RGB);
			// Mat inputImage = rgbThresholdOutput;
			// Imgproc.cvtColor(inputImage, rgb, Imgproc.COLOR_BGR2RGB);
			Core.inRange(rgb, new Scalar(rgbThresholdRed[0], rgbThresholdGreen[0], rgbThresholdBlue[0]),
					new Scalar(rgbThresholdRed[1], rgbThresholdGreen[1], rgbThresholdBlue[1]), rgb);
			// System.out.println("Image Processed");
			Mat kernel = new Mat();
			Imgproc.erode(rgb, rgb, kernel, anchor, (int) times, borderType, borderValue);
			Mat hierarchy = new Mat();
			contours.clear();
			int mode = Imgproc.RETR_LIST;
			int method = Imgproc.CHAIN_APPROX_SIMPLE;
			final MatOfInt hull = new MatOfInt();
			Imgproc.findContours(rgb, contours, hierarchy, mode, method);
			// ArrayList<MatOfPoint> contours1 = contours;
			if (contours.size() > 0) {
				for (int i = 0; i < contours.size(); i++) {
					final MatOfPoint contour = contours.get(i);
					final Rect bb = Imgproc.boundingRect(contours.get(i));
					// didn't work well
					final double area = Imgproc.contourArea(contour);
					double bbWidth = bb.width;
					double bbHeight = bb.height;
					// System.out.println("unfiltered contour number " + i + "'s
					// area is " + area);
					// int contourNumber;
					double ratio = bbHeight / bbWidth;
					if ((ratio < minRatio) || (ratio > maxRatio) && ((area < minArea) || (area > maxArea))) {
						// if( (area < minArea) || (area > maxArea) ){
						System.out.println("contour " + i + "'s ratio is " + ratio);
						System.out.println("Contour does not meet requirements");
						continue;
					}

					// System.out.println(area);
					// final double area = Imgproc.contourArea(contour);
					// else if(area < minArea){
					// break;
					// }
					/*
					 * if(Imgproc.arcLength(new MatOfPoint2f(contour.toArray()),
					 * true) < minPerimeter){ break; }
					 * Imgproc.convexHull(contour, hull);
					 *
					 * MatOfPoint mopHull = new MatOfPoint();
					 * mopHull.create(((int) hull.size().height), 1,
					 * CvType.CV_32SC2); for(int j = 0; j <
					 * (hull.size().height); j++){ int index = (int)hull.get(j,
					 * 0)[0]; double [] point = new double []
					 * {contour.get(index, 0)[0], contour.get(index, 0)[1]};
					 * mopHull.put(j, 0, point); }
					 *
					 * final double solid = (100 * area) /
					 * Imgproc.contourArea(mopHull); else if (solid <
					 * solidity[0] || solid < solidity[1]){ break; }
					 *
					 * else if((contour.rows() < minVertices) || (contour.rows()
					 * > maxVertices)){ break; }
					 *
					 * final double ratio = (bb.width) / ((double)bb.height);
					 * else if ((ratio < minRatio) || (ratio > maxRatio)){
					 * break; }
					 */
					else {
						// final MatOfPoint contours = contours.get(i);
						// contourNumber = i;
						System.out.println("contour " + i + "'s ratio is " + ratio);

						System.out.println("Contour meets requirements!");
						Rect r = Imgproc.boundingRect(contours.get(i));
						double x = bb.x;
						double y = bb.y;
						Point topLeft = new Point(x, y);
						Point bottomRight = new Point(x + bbWidth, y + bbHeight);
						Scalar color = new Scalar(0, 0, 255);
						System.out.println(r);
						Imgproc.rectangle(rgb, topLeft, bottomRight, color);
						imageSource.putFrame(rgb);
						System.out.println("Contour number: " + i + "'s area is " + area);
						System.out.println("Contour meets requirements!");
						u = (r.width / 2);
						v = (r.height / 2);
						// xOverZ = (u - xCenterOfImage)/focalLength;
						// yOverZ = (v - yCenterOfImage)/focalLength;
						// horizontalAngleToTarget = Math.atan((u -
						// xCenterOfImage)/ focalLength );
						// verticalAngleToTarget = Math.atan((v -
						// yCenterOfImage) / focalLength);
						// smartDash.putNumber("horizontal angle to target",
						// horizontalAngleToTarget);
						// smartDash.putNumber("vertical angle to target",
						// verticalAngleToTarget);
						// System.out.println("Horizontal angle to target: " +
						// horizontalAngleToTarget);
						// System.out.println("Vertical angle to target: " +
						// verticalAngleToTarget);
						System.out.println("Calculated stuff that should be on the smartdashboard");

						double apparentWidth = r.width;
						double rayDistance = calculateRayDistance(focalLength, actualWidth, apparentWidth);
						System.out.println("Ray distance is: " + rayDistance);
						double groundDistance = calculateGroundDistance(rayDistance, boilerHeightMinusCameraHeight);
						System.out.println("ground distance is: " + groundDistance);

		double distanceFromEdgeOfImageToContour = r.x;
		double totalDistanceToCenterOfBoundingRect = distanceFromEdgeOfImageToContour + u;
		double remainingDistance = totalDistanceToCenterOfBoundingRect - centerOfImage;
		double distanceStuff = centerOfImage - u; 
		System.out.println("Distance between center of image and center of bounding rect of contour: " + distanceStuff);
		double offset = (remainingDistance - actualWidth) / apparentWidth;
		System.out.println("Offset is: " + offset);
		double angleToTargetInRadians = Math.acos(offset/rayDistance);
		double angleToTargetInDegrees = Math.toDegrees(angleToTargetInRadians);
		System.out.println("Angle to target in radians: " + angleToTargetInRadians);
		System.out.println("Angle to target in degrees " + angleToTargetInDegrees); 


					}
				}
			}
		}

	}

	public static double calculateRayDistance(double focalLength, double actualWidth, double apparentWidth) {
		return ((focalLength * actualWidth) / apparentWidth);
	}

	public static double calculateGroundDistance(double rayDistance, double boilerHeightMinusCameraHeight) {
		return Math.sqrt(Math.pow(rayDistance, 2) - Math.pow(boilerHeightMinusCameraHeight, 2));

	}

	private static UsbCamera setUsbCamera(int cameraId, MjpegServer server) {
		// This gets the image from a USB camera
		// Usually this will be on device 0, but there are other overloads
		// that can be used
		UsbCamera camera = new UsbCamera("CoprocessorCamera", cameraId);
		server.setSource(camera);
		return camera;
	}
}
