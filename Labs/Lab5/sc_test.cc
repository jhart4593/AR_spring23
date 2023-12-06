#include <Eigen/Dense>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>

#include "structurecomputer.h"

int main(int argc, char** argv) { 
// Demonstrate least squares solution based on a measurement model of the form z
// = Hr*x + w, with R = E[w*w'] being the measurement noise covariance matrix
constexpr size_t nx = 3; //dimension of x vector is of dim 3
constexpr size_t nz = 4; //4 meas., 3 unknowns
Eigen::VectorXd z(nz), xHat(nx);//VectorXd lets you specify at time of declaration how big they are
Eigen::MatrixXd Hr(nz, nx), R(nz, nz);
//Fill in z, Hr, R with example values
z << 4,10,6,5; //z was initialized as column vector (this is default for vector sizing)
Hr << 1,0,3,0,2,6,0,0,0,0,0,1e-6;//fills up by columns (column major order)
//std::cout << "Hr \n" << Hr << std::endl;
R = 3*Eigen::MatrixXd::Identity(nz,nz);
Eigen::MatrixXd Rinv = R.inverse();
//The following is the straightforward way to solve a linear least squares problem
//via the normal equations:
xHat = (Hr.transpose()*Rinv*Hr).inverse()*(Hr.transpose()*Rinv*z);
//std::cout << "The straightforward solution is \n" << xHat << std::endl;
// This method is theoretically identical, but numerically more stable:
xHat = (Hr.transpose()*Rinv*Hr).ldlt().solve(Hr.transpose()*Rinv*z);
//std::cout << "The ldlt-based solution is \n" << xHat << std::endl;

using namespace Eigen;
SensorParams sp;
Vector3d rXI;
rXI << 0,0,0.5;
Matrix3d RCB;
RCB << 0,1,0,0.5,0,0.866,0.866,0,-0.5;
//std::cout << "RCB \n" << RCB << std::endl;
Vector3d rI1;
rI1 << 1,0,1;
Matrix3d RBI1;
RBI1 << -1,0,0,0,-1,0,0,0,1;
Vector3d rI2;
rI2 << 0.7071,0.7071,1;
Matrix3d RBI2;
RBI2 << -0.7071,-.7071,0,.7071,-.7071,0,0,0,1;
//std::cout << "RBI2 \n"<< RBI2 << std::endl;
// Create an instance of a StructureComputer object
StructureComputer structureComputer;
// Create shared pointers to two CameraBundle objects. The make_shared function
// creates the objects and returns a shared pointer to it.
// could use 'auto' instead of std::shared_pointer<CameraBundle>
std::shared_ptr<CameraBundle> cb1 = std::make_shared<CameraBundle>();
auto cb2 = std::make_shared<CameraBundle>();
//Fill cb1's and cb2's data members with example contents
cb1->RCI = RCB*RBI1;
cb1->rc_I = rI1+RBI1.transpose()*sp.rc();
cb1->rx = backProject(cb1->RCI,cb1->rc_I,rXI); //use arrow operator not dot because cb1 is not the object but a pointer
cb2->RCI = RCB*RBI2;
cb2->rc_I = rI2+RBI2.transpose()*sp.rc();
cb2->rx = backProject(cb2->RCI,cb2->rc_I,rXI);

structureComputer.clear();
// Push cb1 and cb2 to structureComputer
structureComputer.push(cb1);
structureComputer.push(cb2);

  // Estimate 3D location of feature.  The try and catch blocks are for
  // exception handling -- they handle any errors that might be thrown during a
  // call to computeStructure().  You can use this to protect any portion of
  // your code in main.cc.
  try {
    Point p1 = structureComputer.computeStructure();
  } catch (std::exception& e) {
    std::cout << "Error: " << e.what() << std::endl;
    return EXIT_FAILURE;
  } catch (...) {
    std::cout << "Unhandled error" << std::endl;
    return EXIT_FAILURE;
  }

std::cout << "The actual feature location is \n" << rXI << std::endl;
Point estimate = structureComputer.point();
std::cout << "The estimated feature location is \n" << estimate.rXIHat << std::endl;
Vector3d Pxdiag = estimate.Px.diagonal();
std::cout << "Sqrt of diagonal elements of Px (errors in rXIHat should not be \n"
"much bigger than these numbers) \n" << Pxdiag.cwiseSqrt() << std::endl;

// Display an image
  cv::Mat image = cv::imread(std::string(argv[1]));
  if (!image.data) {
    std::cout << "Could not open or find the image" << std::endl;
    return EXIT_FAILURE;
  }
  cv::namedWindow("Display window", cv::WINDOW_NORMAL);
  cv::resizeWindow("Display window", 600, 600);
  cv::imshow("Display window", image);
  cv::waitKey(0);
  return EXIT_SUCCESS;

return EXIT_SUCCESS; 
}
