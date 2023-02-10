package org.firstinspires.ftc.teamcode.Components.CV;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
@Config
public class SleeveObserverPipeline extends OpenCvPipeline {
    ArrayList<double[]> frameList;
    public static double p1x = 460, p1y =300, p2x = 640, p2y =440,

        h1 = 140,s1 = 0, v1 =0,
            h1u = 160,s1u = 255, v1u =255,
            h2 = 0,s2 = 0, v2 =0,
            h2u =10,s2u = 255, v2u =255,
            h3 = 45,s3 = 60, v3 =0,
            h3u = 60,s3u = 255, v3u =255,

    //h3u and s3u: 71 and 90
            colour = 1;



    public SleeveObserverPipeline() {
        frameList=new ArrayList<>();
    }

    @Override
    public Mat processFrame(Mat input) {
        Rect ROI = new Rect( //130 x 210, 60 x 120
                new Point(p1x,p1y),
                new Point(p2x,p2y));
        Mat mat = new Mat();

        //mat turns into HSV value
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        if (mat.empty()) {
            return input;
        }

        Scalar lowpurpleHSV = new Scalar(h1, s1, v1); //lower bound HSV for purple
        Scalar highpurpleHSV = new Scalar(h1u, s1u, v1u); //higher bound HSV for purple

        Scalar loworangeHSV = new Scalar(h2, s2, v2); //lower bound HSV for orange
        Scalar highorangeHSV = new Scalar(h2u, s2u, v2u); //higher bound HSV for orange

        Scalar lowgreenHSV = new Scalar(h3,s3, v3); //lower bound HSV for green
        Scalar highgreenHSV = new Scalar(h3u, s3u, v3u); //higher bound HSV for green

        Mat thresh = new Mat();

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);


        Core.inRange(mat,lowpurpleHSV,highpurpleHSV,thresh);
        Mat cone = thresh.submat(ROI);
        double purpleValue = Core.sumElems(cone).val[0]/ROI.area()/255;
        Mat thresh2 = new Mat();
        Core.inRange(mat,loworangeHSV,highorangeHSV,thresh2);
        cone.release();
        cone = thresh2.submat(ROI);
        double orangeValue = Core.sumElems(cone).val[0]/ROI.area()/255;
        Mat thresh3 = new Mat();
        Core.inRange(mat,lowgreenHSV,highgreenHSV,thresh3);
        cone.release();
        cone = thresh3.submat(ROI);
        double greenValue = Core.sumElems(cone).val[0]/ROI.area()/255;
        frameList.add(new double[]{orangeValue, greenValue, purpleValue});
        if(frameList.size()>5) {
            frameList.remove(0);
        }

        cone.release();




        //release all the data
        input.release();
        mat.release();
        if(colour ==1){
            thresh.copyTo(input);
        }else if(colour == 2){
            thresh2.copyTo(input);
        }else{
            thresh3.copyTo(input);
        }
        Scalar color = new Scalar(255,0,0);
            Imgproc.rectangle(input, ROI, color, 5);
        thresh.release();
        thresh2.release();
        thresh3.release();
        return input;
    }

   public int getPosition(){
        double[] sums = {0,0,0};
        for(int i=0;i<frameList.size()-1;i++){
            sums[0]+=frameList.get(i)[0];
            sums[1]+=frameList.get(i)[1];
            sums[2]+=frameList.get(i)[2];
        }
//        op.telemetry.addData("sums0",sums[0]);
//       op.telemetry.addData("sums1",sums[1]);
//       op.telemetry.addData("sums2",sums[2]);
//       op.telemetry.update();
           if(sums[0]>sums[1]&&sums[0]>sums[2]){
            return 1;
        }else if(sums[1]>sums[2]){
            return 2;
        }else{
            return 3;
        }
   }
}