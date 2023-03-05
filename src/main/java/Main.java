// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cscore.CvSource;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public final class Main {
  private static String configFile = "/boot/frc.json"; // This is written by the web client when doing camera
                                                       // configuration

  static double METERS_PER_PIXEL = 0.00394137931;
  static boolean DEBUGGING = false;
  
  @SuppressWarnings("MemberName")
  public static class CameraConfig {
    public String name;
    public String path;
    public JsonObject config;
    public JsonElement streamConfig;
  }

  @SuppressWarnings("MemberName")
  public static class SwitchedCameraConfig {
    public String name;
    public String key;
  };

  public static int team;
  public static boolean server;
  public static List<CameraConfig> cameraConfigs = new ArrayList<>();
  public static List<SwitchedCameraConfig> switchedCameraConfigs = new ArrayList<>();
  public static List<VideoSource> cameras = new ArrayList<>();

  private Main() {
  }

  /**
   * Report parse error.
   */
  public static void parseError(String str) {
    System.err.println("config error in '" + configFile + "': " + str);
  }

  /**
   * Read single camera configuration.
   */
  public static boolean readCameraConfig(JsonObject config) {
    CameraConfig cam = new CameraConfig();

    // name
    JsonElement nameElement = config.get("name");
    if (nameElement == null) {
      parseError("could not read camera name");
      return false;
    }
    cam.name = nameElement.getAsString();

    // path
    JsonElement pathElement = config.get("path");
    if (pathElement == null) {
      parseError("camera '" + cam.name + "': could not read path");
      return false;
    }
    cam.path = pathElement.getAsString();

    // stream properties
    cam.streamConfig = config.get("stream");

    cam.config = config;

    cameraConfigs.add(cam);
    return true;
  }

  /**
   * Read single switched camera configuration.
   */
  public static boolean readSwitchedCameraConfig(JsonObject config) {
    SwitchedCameraConfig cam = new SwitchedCameraConfig();

    // name
    JsonElement nameElement = config.get("name");
    if (nameElement == null) {
      parseError("could not read switched camera name");
      return false;
    }
    cam.name = nameElement.getAsString();

    // path
    JsonElement keyElement = config.get("key");
    if (keyElement == null) {
      parseError("switched camera '" + cam.name + "': could not read key");
      return false;
    }
    cam.key = keyElement.getAsString();

    switchedCameraConfigs.add(cam);
    return true;
  }

  /**
   * Read configuration file.
   */
  @SuppressWarnings("PMD.CyclomaticComplexity")
  public static boolean readConfig() {
    // parse file
    JsonElement top;
    try {
      top = new JsonParser().parse(Files.newBufferedReader(Paths.get(configFile)));
    } catch (IOException ex) {
      System.err.println("could not open '" + configFile + "': " + ex);
      return false;
    }

    // top level must be an object
    if (!top.isJsonObject()) {
      parseError("must be JSON object");
      return false;
    }
    JsonObject obj = top.getAsJsonObject();

    // team number
    JsonElement teamElement = obj.get("team");
    if (teamElement == null) {
      parseError("could not read team number");
      return false;
    }
    team = teamElement.getAsInt();

    // ntmode (optional)
    if (obj.has("ntmode")) {
      String str = obj.get("ntmode").getAsString();
      if ("client".equalsIgnoreCase(str)) {
        server = false;
      } else if ("server".equalsIgnoreCase(str)) {
        server = true;
      } else {
        parseError("could not understand ntmode value '" + str + "'");
      }
    }

    // cameras
    JsonElement camerasElement = obj.get("cameras");
    if (camerasElement == null) {
      parseError("could not read cameras");
      return false;
    }
    JsonArray cameras = camerasElement.getAsJsonArray();
    for (JsonElement camera : cameras) {
      if (!readCameraConfig(camera.getAsJsonObject())) {
        return false;
      }
    }

    if (obj.has("switched cameras")) {
      JsonArray switchedCameras = obj.get("switched cameras").getAsJsonArray();
      for (JsonElement camera : switchedCameras) {
        if (!readSwitchedCameraConfig(camera.getAsJsonObject())) {
          return false;
        }
      }
    }

    return true;
  }

  /**
   * Start running the camera.
   */
  public static VideoSource startCamera(CameraConfig config) {
    System.out.println("Starting camera '" + config.name + "' on " + config.path);
    UsbCamera camera = new UsbCamera(config.name, config.path);
    MjpegServer server = CameraServer.startAutomaticCapture(camera);

    Gson gson = new GsonBuilder().create();

    camera.setConfigJson(gson.toJson(config.config));
    camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);

    if (config.streamConfig != null) {
      server.setConfigJson(gson.toJson(config.streamConfig));
    }

    return camera;
  }

  /**
   * Start running the switched camera.
   */
  public static MjpegServer startSwitchedCamera(SwitchedCameraConfig config) {
    System.out.println("Starting switched camera '" + config.name + "' on " + config.key);
    MjpegServer server = CameraServer.addSwitchedCamera(config.name);

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    inst.addListener(
        inst.getTopic(config.key),
        EnumSet.of(NetworkTableEvent.Kind.kImmediate, NetworkTableEvent.Kind.kValueAll),
        event -> {
          if (event.valueData != null) {
            if (event.valueData.value.isInteger()) {
              int i = (int) event.valueData.value.getInteger();
              if (i >= 0 && i < cameras.size()) {
                server.setSource(cameras.get(i));
              }
            } else if (event.valueData.value.isDouble()) {
              int i = (int) event.valueData.value.getDouble();
              if (i >= 0 && i < cameras.size()) {
                server.setSource(cameras.get(i));
              }
            } else if (event.valueData.value.isString()) {
              String str = event.valueData.value.getString();
              for (int i = 0; i < cameraConfigs.size(); i++) {
                if (str.equals(cameraConfigs.get(i).name)) {
                  server.setSource(cameras.get(i));
                  break;
                }
              }
            }
          }
        });

    return server;
  }

  /**
   * Main.
   */
  public static void main(String... args) {



    if (args.length > 0) {
      configFile = args[0];
    }

    // read configuration
    if (!readConfig()) {
      return;
    }

    // start NetworkTables
    NetworkTableInstance ntinst = NetworkTableInstance.getDefault();
    if (server) {
      System.out.println("Setting up NetworkTables server");
      ntinst.startServer();
    } else {
      System.out.println("Setting up NetworkTables client for team " + team);
      ntinst.startClient4("wpilibpi");
      ntinst.setServerTeam(team);
      ntinst.startDSClient();
    }

    // start cameras
    for (CameraConfig config : cameraConfigs) {
      cameras.add(startCamera(config));
    }

    // start switched cameras
    for (SwitchedCameraConfig config : switchedCameraConfigs) {
      startSwitchedCamera(config);
    }

    // Mats are very memory expensive. Lets reuse this Mat.
    // Mat mat = new Mat(640,480,CvType.CV_8UC4);

    CvSource outputStream = CameraServer.putVideo("OPENCV", 640, 480);
    
    
    // start image processing on camera 0 if present
    if (cameras.size() >= 1) {
      VisionThread visionThread = new VisionThread(cameras.get(0),
          new WristCamPipeline(), pipeline -> {
            // do something with pipeline results
            if (!pipeline.findContoursOutput().isEmpty()) {

              // Mat output = new Mat(mat.rows(), mat.cols(), CvType.CV_8UC4);
              Mat mat = pipeline.getOriginalImage();
              // -1 fills in
              for (int i = 0; i < pipeline.convexHullsOutput().size(); i++) {
                Imgproc.drawContours(mat, pipeline.convexHullsOutput(), i, new Scalar(255, 255, 255), 2);
              }

              MatOfPoint2f temp = new MatOfPoint2f();

              temp.fromList(pipeline.convexHullsOutput().get(0).toList());

              // fitline - how about this?
              Mat outputLine = new Mat();
              Imgproc.fitLine(temp, outputLine, Imgproc.DIST_L2, 0, 0.01, 0.01);
              double[] vx = outputLine.get(0, 0);
              double[] vy = outputLine.get(1, 0);
              double[] x = outputLine.get(2, 0);
              double[] y = outputLine.get(3, 0);

              double fitLineAngle = Units.radiansToDegrees(Math.atan(vy[0] / vx[0]));

          

              int lefty = (int) ((-1.0 * x[0] * vy[0] / vx[0]) + y[0]);
              int righty = (int) (((mat.cols() - x[0]) * (vy[0] / vx[0])) + y[0]);

              // Imgproc.line(mat, new Point(mat.cols(), righty), new Point(0, lefty), new Scalar(255, 255, 255), 3);

              Mat triangle = new Mat();
              Imgproc.minEnclosingTriangle(temp, triangle);
              double[] v1 = triangle.get(0, 0);
              double[] v2 = triangle.get(1, 0);
              double[] v3 = triangle.get(2, 0);

              double slopeV1V2 = (v1[1] - v2[1]) / (v1[0] - v2[0]);
              double slopeV2V3 = (v2[1] - v3[1]) / (v2[0] - v3[0]);
              double slopeV3V1 = (v3[1] - v1[1]) / (v3[0] - v1[0]);
              double slopeFitLine = vy[0] / vx[0];

              double triangleAngle1 = Units
                  .radiansToDegrees(Math.atan((slopeV1V2 - slopeFitLine) / (1 + (slopeV1V2 * slopeFitLine))));
              double triangleAngle2 = Units
                  .radiansToDegrees(Math.atan((slopeV2V3 - slopeFitLine) / (1 + (slopeV2V3 * slopeFitLine))));
              double triangleAngle3 = Units
                  .radiansToDegrees(Math.atan((slopeV3V1 - slopeFitLine) / (1 + (slopeV3V1 * slopeFitLine))));

              if(!(triangleAngle1==Double.NaN || triangleAngle2==Double.NaN || triangleAngle3==Double.NaN))
              {
              double baseX1, baseX2, baseY1, baseY2, midX, midY = 0.0;
              double topX, topY = 0.0;

              double TA1 = 0.0;
              double TA2 = 0.0;
              double TA3 = 0.0;

              TA1 = 90.0 - Math.abs(triangleAngle1);
              TA2 = 90.0 - Math.abs(triangleAngle2);
              TA3 = 90.0 - Math.abs(triangleAngle3);

              if ((TA1 < TA2) && (TA1 < TA3)) {
                // V1V2 is the target
                baseX1 = v1[0];
                baseY1 = v1[1];
                baseX2 = v2[0];
                baseY2 = v2[1];
                topX = v3[0];
                topY = v3[1];
                // Imgproc.line(mat, new Point(v2[0], v2[1]), new Point(v3[0], v3[1]), new Scalar(0, 0, 255), 3);
                // Imgproc.line(mat, new Point(v3[0], v3[1]), new Point(v1[0], v1[1]), new Scalar(0, 0, 255), 3);
              } else if ((TA2 < TA1) && (TA2 < TA3)) {
                // V2V3 is the target
                baseX1 = v2[0];
                baseY1 = v2[1];
                baseX2 = v3[0];
                baseY2 = v3[1];
                topX = v1[0];
                topY = v1[1];
                // Imgproc.line(mat, new Point(v1[0], v1[1]), new Point(v2[0], v2[1]), new Scalar(0, 0, 255), 3);
                // Imgproc.line(mat, new Point(v3[0], v3[1]), new Point(v1[0], v1[1]), new Scalar(0, 0, 255), 3);
              } else {
                // V1V3 is the Target
                baseX1 = v1[0];
                baseY1 = v1[1];
                baseX2 = v3[0];
                baseY2 = v3[1];
                topX = v2[0];
                topY = v2[1];
                // Imgproc.line(mat, new Point(v1[0], v1[1]), new Point(v2[0], v2[1]), new Scalar(0, 0, 255), 3);
                // Imgproc.line(mat, new Point(v2[0], v2[1]), new Point(v3[0], v3[1]), new Scalar(0, 0, 255), 3);
              }

              midX = (baseX1 + baseX2) / 2;
              midY = (baseY1 + baseY2) / 2;

              //Imgproc.line(mat, new Point(baseX1, baseY1), new Point(baseX2, baseY2), new Scalar(0, 255, 0), 3); // Draw
                                                                                                                 // Base
                                                                                                                 // Green
              Imgproc.line(mat, new Point(midX, midY), new Point(topX, topY), new Scalar(0, 255, 0), 3); // Draw Mid
                                                                                                         // Line Green

              double newLineAngle = Units.radiansToDegrees(Math.atan((topY - midY) / (topX - midX)));

              double normalLineAngle = newLineAngle; // This is default for quad 1 or 3

              if (topX > midX) {
                // Quad 2 or 4
                if (topY <= midY) {
                  // Quad 2
                  normalLineAngle = 180.0 + newLineAngle; // lineAngle must be a negative number
                } else {
                  // Quad 4
                  normalLineAngle = -180.0 + newLineAngle; // lineAngle must be a positive number
                }
              }


              double coneMidPointX = (midX+topX)/2;
              double coneMidPointY = (midY+topY)/2;
              double coneMidPointXNormal = coneMidPointX-(mat.cols()/2);
              double coneMidPointYNormal = (mat.rows()/2)-coneMidPointY;
            
              double coneMidPointXNormalMeters = coneMidPointXNormal*METERS_PER_PIXEL;
              double coneMidPointYNormalMeters = coneMidPointYNormal*METERS_PER_PIXEL;


              Imgproc.circle(mat, new Point(coneMidPointX, coneMidPointY), 7, new Scalar(255,255,255), -1);

              SmartDashboard.putNumber("Normal Line Angle", normalLineAngle);
              SmartDashboard.putNumber("Cone Mid Point X in Meters", coneMidPointXNormalMeters);
              SmartDashboard.putNumber("Cone Mid Point Y in Meters", coneMidPointYNormalMeters);

              if (DEBUGGING==true)
              {
                SmartDashboard.putNumber("Cone Mid Point X", coneMidPointXNormal);
                SmartDashboard.putNumber("Cone Mid Point Y", coneMidPointYNormal);
                SmartDashboard.putNumber("Triangle Angle 1", (triangleAngle1));
                SmartDashboard.putNumber("Triangle Angle 2", (triangleAngle2));
                SmartDashboard.putNumber("Triangle Angle 3", (triangleAngle3));
                SmartDashboard.putNumber("Fit Line Angle", fitLineAngle);
              }

              // Give the output stream a new image to display
              outputStream.putFrame(mat);
              }
            }
          });

      visionThread.start();
    }

    // loop forever
    for (;;) {
      try {
        Thread.sleep(10000);
      } catch (InterruptedException ex) {
        return;
      }
    }
  }
}
