#include "structurecomputer.h"

#include <Eigen/LU>

void pr(Eigen::MatrixXd m) { std::cout << m << std::endl; }
void pr(Eigen::VectorXd m) { std::cout << m << std::endl; }
void pr(Eigen::Matrix3d m) { std::cout << m << std::endl; }
void pr(Eigen::Vector3d m) { std::cout << m << std::endl; }
void pr(Eigen::Vector2d m) { std::cout << m << std::endl; }

Eigen::Vector2d backProject(const Eigen::Matrix3d& RCI,
                            const Eigen::Vector3d& rc_I,
                            const Eigen::Vector3d& X3d) {
  using namespace Eigen;
  Vector3d t = -RCI * rc_I;
  MatrixXd Pextrinsic(3, 4);
  Pextrinsic << RCI, t;
  SensorParams sp;
  MatrixXd Pc = sp.K() * Pextrinsic;
  VectorXd X(4, 1);
  X.head(3) = X3d;
  X(3) = 1;
  Vector3d x = Pc * X;
  Vector2d xc_pixels = (x.head(2) / x(2)) / sp.pixelSize();
  return xc_pixels;
}

Eigen::Vector3d pixelsToUnitVector_C(const Eigen::Vector2d& rPixels) {
  using namespace Eigen;
  SensorParams sp;
  // Convert input vector to meters
  Vector2d rMeters = rPixels * sp.pixelSize();
  // Write as a homogeneous vector, with a 1 in 3rd element
  Vector3d rHomogeneous;
  rHomogeneous.head(2) = rMeters;
  rHomogeneous(2) = 1;
  // Invert the projection operation through the camera intrinsic matrix K to
  // yield a vector rC in the camera coordinate frame that has a Z value of 1
  Vector3d rC = sp.K().lu().solve(rHomogeneous);
  // Normalize rC so that output is a unit vector
  return rC.normalized();
}

void StructureComputer::clear() {
  // Zero out contents of point_
  point_.rXIHat.fill(0);
  point_.Px.fill(0);
  // Clear bundleVec_
  bundleVec_.clear();
}

void StructureComputer::push(std::shared_ptr<const CameraBundle> bundle) {
  bundleVec_.push_back(bundle);
}

// This function is where the computation is performed to estimate the
// contents of point_.  The function returns a copy of point_.
//
Point StructureComputer::computeStructure() {
  // Throw an error if there are fewer than 2 CameraBundles in bundleVec_,
  // since in this case structure computation is not possible.
  if (bundleVec_.size() < 2) {
    throw std::runtime_error(
        "At least 2 CameraBundle objects are "
        "needed for structure computation.");
  }

  // *********************************************************************
  // Fill in here the required steps to calculate the 3D position of the
  // feature point and its covariance.  Put these respectively in
  // point_.rXIHat and point_.Px
  // *********************************************************************
  using namespace Eigen;
  SensorParams sp;

  int n = bundleVec_.size();
  MatrixXd H(2*n,4);
  for(int ii = 0;ii < n; ii++){
    Vector2d xtilde = bundleVec_[ii]->rx*sp.pixelSize();
    //xtilde[3] = 1;
    Vector3d t = -bundleVec_[ii]->RCI * bundleVec_[ii]->rc_I;
    MatrixXd Pextrinsic(3, 4);
    Pextrinsic << bundleVec_[ii]->RCI, t;
    MatrixXd Pc = sp.K() * Pextrinsic;
    VectorXd P1 = Pc.row(0);
    VectorXd P2 = Pc.row(1);
    VectorXd P3 = Pc.row(2);
    H.row(ii*2) = xtilde[0]*P3 - P1;
    H.row(ii*2+1) = xtilde[1]*P3 - P2;
  }

  MatrixXd Hr = H.block(0,0,2*n,3);
  VectorXd z = -1 * H.block(0,3,2*n,1);

  MatrixXd RC = MatrixXd::Zero(2*n,2*n);
  for(int jj = 0;jj < n;jj++){
    RC.block(jj*2,jj*2,2,2) = sp.Rc();
  }

  MatrixXd R = pow(sp.pixelSize(),2)*RC;
  constexpr size_t nx = 3;
  VectorXd xHat(nx);
  MatrixXd Px(nx,nx);
  MatrixXd Rinv = R.inverse();
  xHat = (Hr.transpose()*Rinv*Hr).ldlt().solve(Hr.transpose()*Rinv*z);
  Px = (Hr.transpose()*Rinv*Hr).inverse();

  point_.rXIHat = xHat;
  point_.Px = Px;

  return point_;
}