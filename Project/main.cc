#include <dirent.h>

#include <Eigen/Dense>
#include <boost/program_options.hpp>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <cmath>
#include <algorithm>

#include "balloonfinder.h"
#include "imagemetadata.h"
#include "navtoolbox.h"
#include "structurecomputer.h"

namespace po = boost::program_options;

int main(int argc, char** argv) {
  // Set defaults and draw in command-line arguments
  std::string imageDirectory = "../images/tournament-train";
  Eigen::Vector3d blueTrue_I, redTrue_I;
  // True balloon locations
  blueTrue_I << -22.6496782957487, 17.8542829259753, -3.58;
  redTrue_I << -17.7320426477925, 15.5995364648455, -3.58;
  bool debuggingEnabled = false;
  bool calibrationEnabled = false;
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("images,i", po::value<std::string>(&imageDirectory),
     "directory where images are located")
    ("debug,d", po::bool_switch(&debuggingEnabled),
     "enable interactive debugging")
    ("calibrate,c", po::bool_switch(&calibrationEnabled),
     "enable calibration of camera extrinsics");
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return EXIT_FAILURE;
  }

  // Find all jpg file names in the input directory and store these in
  // imageFilenameVec
  std::vector<std::string> imageFilenameVec;
  DIR* directory;
  dirent* entries;
  if ((directory = opendir(imageDirectory.c_str())) != nullptr) {
    while ((entries = readdir(directory)) != nullptr) {
      std::string imageFilename(entries->d_name);
      size_t found = imageFilename.find("jpg");
      if (found != std::string::npos) {
        imageFilenameVec.push_back(imageFilename);
      }
    }
  } else {
    std::cerr
        << "Error: Could not find specified directory where images are located."
        << std::endl;
    return EXIT_FAILURE;
  }
  std::sort(imageFilenameVec.begin(), imageFilenameVec.end());

  // Read in the image metadata and store in a std::map for easy lookup
  std::ifstream metadata(imageDirectory + "/metadata.log");
  if (!metadata.is_open()) {
    std::cerr << "Error: A metadata file named metadata.log is expected"
              << " to be found in the input directory." << std::endl;
    return EXIT_FAILURE;
  }
  std::map<std::string, ImageMetadata> imageMetadataMap;
  std::string metadataLine;
  // Discard first line, which merely describes data format
  std::getline(metadata, metadataLine);
  // Push subsequent lines' data into map
  while (std::getline(metadata, metadataLine)) {
    ImageMetadata imageMetadata(metadataLine);
    imageMetadataMap[imageMetadata.filename()] = imageMetadata;
  }

  // The try and catch blocks are for exception (e.g., error) handling
  try {
    // Extract balloon center coordinates from images.
    BalloonFinder balloonFinder(debuggingEnabled, calibrationEnabled,
                                blueTrue_I, redTrue_I);
    StructureComputer structureComputerBlue, structureComputerRed;
    // Establish vectors to store all red and blue camera bundle objects
    std::vector<std::shared_ptr<const CameraBundle>> redBundles;
    std::vector<std::shared_ptr<const CameraBundle>> blueBundles;
    // Draw in and process each image
    for (const auto& imageFilename : imageFilenameVec) {
      std::string imagePath = imageDirectory + "/" + imageFilename;
      cv::Mat image = cv::imread(imagePath);
      if (!image.data) {
        std::cout << "Could not read file " << imagePath << std::endl;
        return EXIT_FAILURE;
      }
      // Retrieve the metadata for this image
      const auto& imd = imageMetadataMap.at(imageFilename);
      // Send image to balloonFinder
      std::vector<std::shared_ptr<const CameraBundle>> bundles;
      std::vector<BalloonFinder::BalloonColor> colors;
      std::cout << "\nProcessing image " << imageFilename << std::endl;
      balloonFinder.findBalloons(&image, imd.RCI(), imd.rc_I(), &bundles,
                                 &colors);
      for (size_t ii = 0; ii < bundles.size(); ii++) {
        switch (colors[ii]) {
          case BalloonFinder::BalloonColor::RED:
            //structureComputerRed.push(bundles[ii]);
            redBundles.push_back(bundles[ii]);
            break;
          case BalloonFinder::BalloonColor::BLUE:
            //structureComputerBlue.push(bundles[ii]);
            blueBundles.push_back(bundles[ii]);
            break;
          default:
            break;
        }
      }

      
    }

    //std::cout << redBundles.size() << std::endl;

    //Set RANSAC parameters
    size_t numCandidateSolns = 100;
    double dmax = 100; //in pixels 

    //pull pairs randomly from cam bundle vectors with replacement
    //compute structure for each pair and store in vector
    cv::RNG rng(12345);
    StructureComputer structureComputerRedPair, structureComputerBluePair; 
    size_t redBundleSize = redBundles.size()-1;
    size_t blueBundleSize = blueBundles.size()-1;
    std::vector<Point> RedCandSolns;
    std::vector<Point> BlueCandSolns;

    for (size_t jj = 0; jj < numCandidateSolns; jj++){
      structureComputerRedPair.push(redBundles[rng.uniform(0,redBundleSize)]);
      structureComputerRedPair.push(redBundles[rng.uniform(0,redBundleSize)]);
      structureComputerBluePair.push(blueBundles[rng.uniform(0,blueBundleSize)]);
      structureComputerBluePair.push(blueBundles[rng.uniform(0,blueBundleSize)]);

      Point pRedPair = structureComputerRedPair.computeStructure();
      Point pBluePair = structureComputerBluePair.computeStructure();

      RedCandSolns.push_back(pRedPair);
      BlueCandSolns.push_back(pBluePair);
    }

    // Re-project 3D location for each candidate solution onto the
    // images corresponding to the camera bundles. Measure the distance from 
    // measured balloon center to decide if camera bundle offers support for
    // this candidate solution.
    // Red Loop
    std::vector<std::vector<std::shared_ptr<const CameraBundle>>> redVotes;
    for (auto candidate_soln : RedCandSolns){
      std::vector<std::shared_ptr<const CameraBundle>> redBundleSupport;
      for (auto bundle : redBundles){
        Eigen::Vector2d xc = backProject(bundle->RCI,bundle->rc_I,candidate_soln.rXIHat);
        double d = sqrt(pow(abs(xc(0)-bundle->rx(0)),2) + pow(abs(xc(1)-bundle->rx(1)),2));
        if (d <= dmax){
          redBundleSupport.push_back(bundle);
        }
      }
      redVotes.push_back(redBundleSupport);
    }

    // Blue Loop
    std::vector<std::vector<std::shared_ptr<const CameraBundle>>> blueVotes;
    for (auto candidate_soln : BlueCandSolns){
      std::vector<std::shared_ptr<const CameraBundle>> blueBundleSupport;
      for (auto bundle : blueBundles){
        Eigen::Vector2d xc = backProject(bundle->RCI,bundle->rc_I,candidate_soln.rXIHat);
        double d = sqrt(pow(abs(xc(0)-bundle->rx(0)),2) + pow(abs(xc(1)-bundle->rx(1)),2));
        if (d <= dmax){
          blueBundleSupport.push_back(bundle);
        }
      }
      blueVotes.push_back(blueBundleSupport);
    }

    // Choose candidate solution with greatest support among camera bundles
    std::vector<double> redVotesSize;
    for (auto vec : redVotes){
      redVotesSize.push_back(vec.size());
    }
    int redIndex = std::max_element(redVotesSize.begin(),redVotesSize.end()) - redVotesSize.begin();
    std::vector<std::shared_ptr<const CameraBundle>> redWinner = redVotes[redIndex];

    std::vector<double> blueVotesSize;
    for (auto vec : blueVotes){
      blueVotesSize.push_back(vec.size());
    }
    int blueIndex = std::max_element(blueVotesSize.begin(),blueVotesSize.end()) - blueVotesSize.begin();
    std::vector<std::shared_ptr<const CameraBundle>> blueWinner = blueVotes[blueIndex];

    // Push winning candidate solution camera bundle supporters
    // into blue and red structure computers
    for (auto bundle : redWinner){
      structureComputerRed.push(bundle);
    }

    for (auto bundle : blueWinner){
      structureComputerBlue.push(bundle);
    }

    // Estimate 3D location of feature.
    Point pBlue = structureComputerBlue.computeStructure();
    Point pRed = structureComputerRed.computeStructure();
    std::cout << "\nBlue balloon estimated 3D location error: \n"
              << pBlue.rXIHat - blueTrue_I << std::endl;
    std::cout << "Red balloon estimated 3D location error: \n"
              << pRed.rXIHat - redTrue_I << std::endl;
    std::cout << "Blue error covariance matrix sqrt diagonal: \n"
              << pBlue.Px.diagonal().cwiseSqrt() << std::endl;
    std::cout << "Red error covariance matrix sqrt diagonal: \n"
              << pRed.Px.diagonal().cwiseSqrt() << std::endl;

    if (calibrationEnabled) {
      // Output calibrated eCB
      const Eigen::Vector3d eCB_rad = balloonFinder.eCB_calibrated();
      std::cout << "eCB_calibrated (deg): " << eCB_rad.transpose() * 180 / PI
                << std::endl;
    }
  } catch (std::exception& e) {
    std::cout << "Error: " << e.what() << std::endl;
    return EXIT_FAILURE;
  } catch (...) {
    std::cout << "Unhandled error" << std::endl;
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
