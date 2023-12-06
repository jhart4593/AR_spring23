#include "balloonfinder.h"

#include <Eigen/Dense>
#include <cassert>
#include <opencv2/core/eigen.hpp>

#include "navtoolbox.h"

BalloonFinder::BalloonFinder(bool debuggingEnabled, bool calibrationEnabled,
                             const Eigen::Vector3d& blueTrue_I,
                             const Eigen::Vector3d& redTrue_I) {
  debuggingEnabled_ = debuggingEnabled;
  calibrationEnabled_ = calibrationEnabled;
  blueTrue_I_ = blueTrue_I;
  redTrue_I_ = redTrue_I;
  V_.resize(3, 0);
  W_.resize(3, 0);
}

// Returns true if the input contour touches the edge of the input image;
// otherwise returns false.
//
bool touchesEdge(const cv::Mat& image, const std::vector<cv::Point>& contour) {
  const size_t borderWidth = static_cast<size_t>(0.01 * image.rows);

  for (const auto& pt : contour) {
    if (pt.x <= borderWidth || pt.x >= (image.cols - borderWidth) ||
        pt.y <= borderWidth || pt.y >= (image.rows - borderWidth))
      return true;
  }
  return false;
}

Eigen::Vector3d BalloonFinder::eCB_calibrated() const {
  using namespace Eigen;
  const SensorParams sp;
  const size_t N = V_.cols();
  if (N < 2 || !calibrationEnabled_) {
    return Vector3d::Zero();
  }
  const VectorXd aVec = VectorXd::Ones(N);
  const Matrix3d dRCB = navtbx::wahbaSolver(aVec, W_, V_);
  const Matrix3d RCB = navtbx::euler2dc(sp.eCB());
  return navtbx::dc2euler(dRCB * RCB);
}

bool BalloonFinder::findBalloonsOfSpecifiedColor(
    const cv::Mat* image, const Eigen::Matrix3d RCI, const Eigen::Vector3d rc_I,
    const BalloonFinder::BalloonColor color,
    std::vector<Eigen::Vector2d>* rxVec) {
  using namespace cv;
  bool returnValue = false;
  rxVec->clear();
  Mat original, originalHSV;
  if (debuggingEnabled_) {
    original = image->clone(); 
    //originalHSV = image->clone();
    //cvtColor(originalHSV, originalHSV, COLOR_BGR2HSV);
  }
  const size_t nCols_m1 = image->cols - 1;
  const size_t nRows_m1 = image->rows - 1;
  //std::cout << "columns: " << nCols_m1 << std::endl;
  //std::cout << "rows: " << nRows_m1 << std::endl;
  // Blur the image to reduce small-scale noise
  Mat framep;
  GaussianBlur(*image, framep, Size(21, 21), 0, 0);

  // *************************************************************************
  //
  // Implement the rest of the function here.  Your goal is to find a balloon
  // of the color specified by the input 'color', and find its center in image
  // plane coordinates (see the comments below for a discussion on image plane
  // coordinates), expressed in pixels.  Suppose rx is an Eigen::Vector2d
  // object that holds the x and y position of a balloon center in such
  // coordinates.  You can push rx onto rxVec as follows: rxVec->push_back(rx)
  //
  // *************************************************************************

  cvtColor(framep, framep, COLOR_BGR2HSV);
  Scalar balloonColor;
  std::string balloon_color;
  switch(color){
    case BalloonColor::RED:{
      Scalar colorLower_l(0, 120, 100), colorLower_h(10,255,255);
      Scalar colorUpper_l(170, 120, 100), colorUpper_h(180,255,255);
      Mat mLower, mUpper; //image can be thought of as matrix of pixels
      //mLower and mUpper will store 1 for every pixel in their respective in ranges
      inRange(framep,colorLower_l,colorLower_h,mLower);
      inRange(framep,colorUpper_l,colorUpper_h,mUpper);
      framep = mLower | mUpper; //bit wise 'or' both matrices
      balloonColor = Scalar(0,0,255);
      balloon_color = "Red";
      break;
    }
    case BalloonColor::BLUE:{
      Scalar lower_blue(80,140,165), upper_blue(103,255,255);
      Mat mOut;
      inRange(framep,lower_blue,upper_blue,mOut);
      framep = mOut;  
      balloonColor = Scalar(255,0,0);
      balloon_color = "Blue";
      break;
    }
    default:{
      throw std::runtime_error("Color value is not handled in this program.");
    }
  }

  /*namedWindow("pre", WINDOW_NORMAL);
  resizeWindow("pre", 1000, 1000);
  imshow("pre", *image);*/

  // Erode image to eliminate stray wisps of color
  constexpr int iterations = 10;
  erode(framep, framep, Mat(), cv::Point(-1,-1), iterations);
  // Dilate image to restore balloon to original size
  dilate(framep, framep, Mat(), cv::Point(-1,-1), iterations);

  /*namedWindow("post", WINDOW_NORMAL);
  resizeWindow("post", 1000, 1000);
  imshow("post", framep);
  waitKey(0);*/

  // Find contours
  std::vector<std::vector<cv::Point>> contours;
  std::vector<Vec4i> hierarchy;
  findContours(framep, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
  // Loop through the contours.  Bound each contour by both a bounding
  // (rotated) rectangle and a minimum enclosing circle.  Draw the contour on
  // the original image.  If the bounding rectangle has aspect ratio close to balloon
  // and if the enclosing circle is large enough, then also
  // draw the minimum enclosing circle.
  RNG rng(12345);
  Point2f center;
  float radius;
  float aspectRatio;
  constexpr float maxAspectRatio = 1.45;
  constexpr float minAspectRatio = 1.18;
  constexpr float maxRadius = 300;
  constexpr float minRadius = 50;
  constexpr int minPointsFor_fitEllipse = 5;

  if (debuggingEnabled_){
    std::cout << balloon_color << " testing:" << std::endl;
  }

  for (size_t ii = 0; ii < contours.size(); ii++) {
    if (touchesEdge(framep, contours[ii])){
      continue;
    }
    const Scalar color =
        Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
    minEnclosingCircle(contours[ii], center, radius);
    aspectRatio = maxAspectRatio;
    if (contours[ii].size() >= minPointsFor_fitEllipse) {
      RotatedRect boundingRectangle = fitEllipse(contours[ii]);
      const Size2f rectSize = boundingRectangle.size;
      aspectRatio =
          static_cast<float>(std::max(rectSize.width, rectSize.height)) /
          std::min(rectSize.width, rectSize.height);
    }

    if (aspectRatio < maxAspectRatio && aspectRatio > minAspectRatio 
      && radius > minRadius && radius < maxRadius) {

      if (debuggingEnabled_){  
        circle(original, center, static_cast<int>(radius), balloonColor, 8);
        circle(original, center, static_cast<int>(radius/6), balloonColor, 8);
        std::cout << balloon_color << " balloon found" << ", cx: " <<
        center.x << ", cy: " << center.y << std::endl;
        //cv::Vec3b& centerHSV = originalHSV.at<cv::Vec3b>(center.x,center.y);
        //std::cout << "Center HSV: " << centerHSV[0] << ",";
        //std::cout << centerHSV[1] << ",";
        //std::cout << centerHSV[2] << std::endl;
      }

      Eigen::Vector2d rx;
      rx(0) = nCols_m1 - center.x;
      rx(1) = nRows_m1 - center.y;
      rxVec->push_back(rx);
      returnValue = true;
    }

    if (debuggingEnabled_){
      drawContours(original, contours, ii, color, 8, LINE_8, hierarchy, 0);
      std::cout << "aspectRatio: " << aspectRatio << ", radius: " << radius << std::endl;
    }

  }


  // The debugging section below plots the back-projection of true balloon 3d
  // location on the original image.  The balloon centers you find should be
  // close to the back-projected coordinates in xc_pixels.  Feel free to alter
  // the debugging section below, or add other such sections, so you can see
  // how your found centers compare with the back-projected centers.
  if (debuggingEnabled_) {
    // Clone the original image for debugging purposes
    //original = image->clone();
    Eigen::Vector2d xc_pixels;
    Scalar trueProjectionColor;
    if (color == BalloonColor::BLUE) {
      xc_pixels = backProject(RCI, rc_I, blueTrue_I_);
      trueProjectionColor = Scalar(255, 0, 0);
    } else {
      xc_pixels = backProject(RCI, rc_I, redTrue_I_);
      trueProjectionColor = Scalar(0, 0, 255);
    }

    Point2f true_center;
    // The image plane coordinate system, in which xc_pixels is expressed, has
    // its origin at the lower-right of the image, x axis pointing left and y
    // axis pointing up, whereas the variable 'center' below, used by OpenCV
    // for plotting on the image, is referenced to the image's top left corner
    // and has the opposite x and y directions.  The measurements returned in
    // rxVec should be given in the image plane coordinate system like
    // xc_pixels.  Hence, once you've found a balloon center from your image
    // processing techniques, you'll need to convert it to the image plane
    // coordinate system using an inverse of the mapping below.
    true_center.x = nCols_m1 - xc_pixels(0);
    true_center.y = nRows_m1 - xc_pixels(1);
    circle(original, true_center, 10, trueProjectionColor, FILLED);
    namedWindow("Display", WINDOW_NORMAL);
    resizeWindow("Display", 1000, 1000);
    imshow("Display", original);

    //namedWindow("Display1", WINDOW_NORMAL);
    //resizeWindow("Display1", 1000, 1000);
    //imshow("Display1", framep);
    if (returnValue){
      std::cout << "true cx: " << true_center.x << ", true cy: " << true_center.y << std::endl; 
    }

    waitKey(0);
  }

  
  return returnValue;
}

void BalloonFinder::findBalloons(
    const cv::Mat* image, const Eigen::Matrix3d RCI, const Eigen::Vector3d rc_I,
    std::vector<std::shared_ptr<const CameraBundle>>* bundles,
    std::vector<BalloonColor>* colors) {
  // Crop image to 4k size.  This removes the bottom 16 rows of the image,
  // which are an artifact of the camera API.
  const cv::Rect croppedRegion(0, 0, sensorParams_.imageWidthPixels(),
                               sensorParams_.imageHeightPixels());
  cv::Mat croppedImage = (*image)(croppedRegion);
  // Convert camera instrinsic matrix K and distortion parameters to OpenCV
  // format
  cv::Mat K, distortionCoeffs, undistortedImage;
  Eigen::Matrix3d Kpixels = sensorParams_.K() / sensorParams_.pixelSize();
  Kpixels(2, 2) = 1;
  cv::eigen2cv(Kpixels, K);
  cv::eigen2cv(sensorParams_.distortionCoeffs(), distortionCoeffs);
  // Undistort image
  cv::undistort(croppedImage, undistortedImage, K, distortionCoeffs);

  // Find balloons of specified color
  std::vector<BalloonColor> candidateColors = {BalloonColor::RED,
                                               BalloonColor::BLUE};
  for (auto color : candidateColors) {
    std::vector<Eigen::Vector2d> rxVec;
    if (findBalloonsOfSpecifiedColor(&undistortedImage, RCI, rc_I, color,
                                     &rxVec)) {
      for (const auto& rx : rxVec) {
        std::shared_ptr<CameraBundle> cb = std::make_shared<CameraBundle>();
        cb->RCI = RCI;
        cb->rc_I = rc_I;
        cb->rx = rx;
        bundles->push_back(cb);
        colors->push_back(color);
      }
    }
  }
}
