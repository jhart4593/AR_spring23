#include <chrono>

#include "student_autonomy_protocol.h"
#include "graph.h"
#include "occupancy_grid3d.h"
#include "autonomy_protocol_visualizer.h"
#include "polynomial_solver.h"
#include "polynomial_sampler.h"
#include "a_star3d.h"
#include "pruning.h"

// The length of one side of a cube in meters in the occupancy grid
constexpr double DISCRETE_LENGTH = 0.2;
// How big the "inflation" bubble around obstacles will be, in meters
constexpr double SAFETY_BOUNDS = 0.5;

namespace game_engine {
std::chrono::milliseconds dt_chrono = std::chrono::milliseconds(10);

// UpdateTrajectories creates and returns a proposed trajectory.  The proposed
// trajectory gets submitted to the mediation_layer (ML), which responds by
// setting the data member trajectoryCodeMap_.  See the header file
// game-engine/src/util/trajectory_code.h for a list of possible codes.
//
// Any code other than MediationLayer::Success indicates that the ML has
// rejected the submitted trajectory.
//
// trajectoryCodeMap_ is initialized with MediationLayerCode::Success, so this
// will be its value the first time this function is called (before any
// trajectories have been submitted).  Thereafter, trajectoryCodeMap_ will
// indicate the MediationLayerCode for the most recently submitted trajectory.
std::unordered_map<std::string, Trajectory>
StudentAutonomyProtocol::UpdateTrajectories() {
  // Set the duration of the example trajectory
  constexpr int duration_sec = 300;
  const std::chrono::milliseconds T_chrono = std::chrono::seconds(duration_sec);

  // 'static' variables act like Matlab persistent variables, maintaining their
  // value between function calls. Their initializer is only called once, on the
  // first pass through the code.  If you prefer not to include static
  // variables, you could instead make these data members of your
  // StudentAutonomyProtocol class, which is a more standard C++ code design
  // pattern.
  static OccupancyGrid3D occupancy_grid;
  static Graph3D graph_of_arena;
  // Student_game_engine_visualizer is a class that supports visualizing paths,
  // curves, points, and whole trajectories in the RVIZ display of the arena to
  // aid in your algorithm development.
  static AutonomyProtocolVisualizer visualizer;
  static bool first_time = true;
  static bool halt = false;
  static Eigen::Vector3d start_pos;
  static const std::chrono::time_point<std::chrono::system_clock>
      start_chrono_time = std::chrono::system_clock::now();
  static const std::chrono::time_point<std::chrono::system_clock>
      end_chrono_time = start_chrono_time + T_chrono;

  // Load the current quad position
  const std::string& quad_name = friendly_names_[0];
  Eigen::Vector3d current_pos;
  Eigen::Vector3d current_vel;
  snapshot_->Position(quad_name, current_pos);
  snapshot_->Velocity(quad_name, current_vel);
//  std::cout << "Current position: " << current_pos << std::endl;

  // Set some static variables the first time this function is called
  if (first_time) {
    first_time = false;
    occupancy_grid.LoadFromMap(map3d_, DISCRETE_LENGTH, SAFETY_BOUNDS);
    // You can run A* on graph_of_arena once you created a 3D version of A*
    graph_of_arena = occupancy_grid.AsGraph();
    visualizer.startVisualizing("/game_engine/environment");
    start_pos = current_pos;
  }

  // Obtain current balloon positions and popped states
  const Eigen::Vector3d red_balloon_pos = *red_balloon_position_;
  const Eigen::Vector3d blue_balloon_pos = *blue_balloon_position_;
  static Eigen::Vector3d prev_red_balloon_pos = red_balloon_pos;
  static Eigen::Vector3d prev_blue_balloon_pos = blue_balloon_pos;
  const bool red_balloon_popped = red_balloon_status_->popped;
  const bool blue_balloon_popped = blue_balloon_status_->popped;

  // Create an empty quad-to-trajectory map.  This map object associates a quad
  // name (expressed as a std::string) with the corresponding Trajectory object.
  std::unordered_map<std::string, Trajectory> quad_to_trajectory_map;

  // Condition actions or parameters on wind intensity
  switch (wind_intensity_) {
    case WindIntensity::Zero:
      // Do something zero-ish
      break;
    case WindIntensity::Mild:
      // Do something mild
      break;
    case WindIntensity::Stiff:
      // Do something stiff
      break;
    case WindIntensity::Intense:
      // Do something intense
      break;
    case WindIntensity::Ludicrous:
      // Do something ludicrous
      break;
    default:
      std::cerr << "Unrecognized WindIntensity value." << std::endl;
      std::exit(EXIT_FAILURE);
  }
/*
  // You can fill out this switch statement with case statements tailored to
  // each MediationLayerCode.
  switch (trajectoryCodeMap_[quad_name].code) {
    case MediationLayerCode::Success:
      // You probably won't need to do anything in response to Success.
      break;
    case MediationLayerCode::TimeBetweenPointsExceedsMaxTime: {
      // Suppose your AP initially submits a trajectory with a time that exceeds
      // the maximum allowed time between points. You could fix the problem as
      // shown below.
      std::cout << "Replanning trajectory: "
                   "Shortening time between trajectory points."
                << std::endl;
      std::cout << "Value: " << trajectoryCodeMap_[quad_name].value
                << std::endl;
      dt_chrono = dt_chrono - std::chrono::milliseconds(5);
      break;
    }
    default:
      // If you want to see a numerical MediationLayerCode value, you can cast
      // and print the code as shown below.
      std::cout << "MediationLayerCode: "
                << static_cast<int>(trajectoryCodeMap_[quad_name].code)
                << std::endl;
      std::cout << "Value: " << trajectoryCodeMap_[quad_name].value
                << std::endl;
      std::cout << "Index: " << trajectoryCodeMap_[quad_name].index
                << std::endl;
  }
*/
  // Always use the chrono::system_clock for time. Trajectories require time
  // points measured in floating point seconds from the Unix epoch.
  const double dt =
      std::chrono::duration_cast<std::chrono::duration<double>>(dt_chrono)
          .count();
  std::chrono::time_point<std::chrono::system_clock> current_chrono_time =
      std::chrono::system_clock::now();
  const std::chrono::duration<double> remaining_chrono_time =
      end_chrono_time - current_chrono_time;

  // The following code generates and returns a new trajectory each time it
  // runs.  The new trajectory starts at the location on the original circle
  // that is closest to the current location of the quad and it creates a
  // trajectory of N position, velocity, and acceleration (PVA) points spaced by
  // dt seconds.  Thus, the code below responds to the actual position of the
  // quad and adjusts the newly-generated trajectory accordingly.  But note that
  // its strategy is not optimal for covering the greatest distance in the
  // allotted time in the presence of disturbance accelerations.

  // Number of samples in new trajectory
  const size_t N = remaining_chrono_time / dt_chrono;

  // If the end time has passed, or if there are too few samples for a valid
  // trajectory, return an empty quad_to_trajectory_map
  constexpr size_t min_required_samples_in_trajectory = 2;
  if (current_chrono_time >= end_chrono_time ||
      N < min_required_samples_in_trajectory) {
    return quad_to_trajectory_map;
  }

  // Halt at goal position when close enough
  constexpr double goal_arrival_threshold_meters = 0.3;
  const Eigen::Vector3d dv = current_pos - goal_position_;
  // TrajectoryVector3D is an std::vector object defined in trajectory.h
  TrajectoryVector3D trajectory_vector;
  if (halt ||
      (remaining_chrono_time < std::chrono::seconds(duration_sec - 10) &&
       dv.norm() < goal_arrival_threshold_meters)) {
    halt = true;
    for (size_t idx = 0; idx < 20; ++idx) {
      const std::chrono::duration<double> flight_chrono_time =
          current_chrono_time.time_since_epoch() + idx * dt_chrono;
      const double flight_time = flight_chrono_time.count();
      trajectory_vector.push_back(
          (Eigen::Matrix<double, 11, 1>() << goal_position_.x(),
           goal_position_.y(), goal_position_.z(), 0, 0, 0, 0, 0, 0, 0,
           flight_time)
              .finished());
    }
    Trajectory trajectory(trajectory_vector);
    visualizer.drawTrajectory(trajectory);
    quad_to_trajectory_map[quad_name] = trajectory;
    return quad_to_trajectory_map;
  }

  static bool firstcalled = false;
  static bool midcalled = false;
  static bool lastcalled = false;

  Eigen::Vector3d red_diff = prev_red_balloon_pos - red_balloon_pos;
  Eigen::Vector3d blue_diff = prev_blue_balloon_pos - blue_balloon_pos;
  if((prev_red_balloon_pos - red_balloon_pos).norm() > 0.01 || (prev_blue_balloon_pos - blue_balloon_pos).norm() > 0.01){
    std::cout << "A balloon teleported, recalculating trajectory..." << std::endl;
    prev_red_balloon_pos = red_balloon_pos;
    prev_blue_balloon_pos = blue_balloon_pos;
    firstcalled = false;
    midcalled = false;
    lastcalled = false;
    for (size_t idx = 0; idx < 20; ++idx) {
      const std::chrono::duration<double> flight_chrono_time =
          current_chrono_time.time_since_epoch() + idx * dt_chrono;
      const double flight_time = flight_chrono_time.count();
      trajectory_vector.push_back(
          (Eigen::Matrix<double, 11, 1>() << goal_position_.x(),
           goal_position_.y(), goal_position_.z(), 0, 0, 0, 0, 0, 0, 0,
           flight_time)
              .finished());
    }
    Trajectory trajectory(trajectory_vector);
    visualizer.drawTrajectory(trajectory);
    quad_to_trajectory_map[quad_name] = trajectory;
    return quad_to_trajectory_map;
  }

  // See which balloon should be targeted first, based on proximity
  Eigen::Vector3d target_pos;
  if (!red_balloon_popped && !blue_balloon_popped) {
    if (firstcalled){
         TrajectoryVector3D trajectory_vector;
         Trajectory trajectory(trajectory_vector);
         quad_to_trajectory_map[quad_name] = trajectory;
         return quad_to_trajectory_map;
    }
    firstcalled = true;
    if ((red_balloon_pos - current_pos).norm() <
        (blue_balloon_pos - current_pos).norm()) {
      target_pos = red_balloon_pos;
      std::cout << "Red balloon closer, targeting red balloon" << std::endl;
      //std::cout << "Red balloon position: " << red_balloon_pos << std::endl;
    } else {
      target_pos = blue_balloon_pos;
      std::cout << "Targeting blue balloon" << std::endl;
    }
  } else if (!red_balloon_popped && blue_balloon_popped) {
    if (midcalled){
      TrajectoryVector3D trajectory_vector;
      Trajectory trajectory(trajectory_vector);
      quad_to_trajectory_map[quad_name] = trajectory;
      return quad_to_trajectory_map;
    }   
    midcalled = true;
    target_pos = red_balloon_pos;
    std::cout << "Blue popped, targeting red balloon" << std::endl;
  } else if (!blue_balloon_popped && red_balloon_popped) {
    if (midcalled){
      TrajectoryVector3D trajectory_vector;
      Trajectory trajectory(trajectory_vector);
      quad_to_trajectory_map[quad_name] = trajectory;
      return quad_to_trajectory_map;
    }   
    midcalled = true;
    target_pos = blue_balloon_pos;
    std::cout << "Red popped, targeting blue balloon" << std::endl;
  } else{
    // Both balloons have been popped
    if (lastcalled){
      TrajectoryVector3D trajectory_vector;
      Trajectory trajectory(trajectory_vector);
      quad_to_trajectory_map[quad_name] = trajectory;
      return quad_to_trajectory_map;
    }   
    lastcalled=true;
    target_pos = goal_position_;
    std::cout << "Both balloons popped, heading to goal" << std::endl;
  }

  // Use A* to create a trajectory from the current position to the target position
 
  std::tuple<int, int, int> current_pos_tuple = occupancy_grid.mapToGridCoordinates(current_pos);
  std::tuple<int, int, int> target_pos_tuple = occupancy_grid.mapToGridCoordinates(target_pos);

  std::cout << "Current position: " << current_pos << std::endl;
  std::cout << "Target position: " << target_pos << std::endl;

  Eigen::Vector3d current_pos_index;
  current_pos_index(0) = std::get<0>(current_pos_tuple);
  current_pos_index(1) = std::get<1>(current_pos_tuple);
  current_pos_index(2) = std::get<2>(current_pos_tuple);

  std::cout << "current_pos_index: " << current_pos_index << std::endl;

  Eigen::Vector3d target_pos_index;
  target_pos_index(0) = std::get<0>(target_pos_tuple);
  target_pos_index(1) = std::get<1>(target_pos_tuple);
  target_pos_index(2) = std::get<2>(target_pos_tuple);

  std::cout << "target_pos_index: " << target_pos_index << std::endl;

  std::shared_ptr<Node3D> start_ptr = std::make_shared<Node3D>(current_pos_index);
  std::shared_ptr<Node3D> end_ptr = std::make_shared<Node3D>(target_pos_index);
  AStar3D a_star;
  PathInfo path_info = a_star.Run(graph_of_arena, start_ptr, end_ptr);

  path_info.details.Print(); //For debugging

  // prune Astar path to remove extra waypoints
  for(int ii = 0; ii < path_info.details.path_length-2; ii++){
    Eigen::Vector3d startcoords = {path_info.path[ii]->Data().transpose() [0], 
      path_info.path[ii]->Data().transpose() [1], path_info.path[ii]->Data().transpose() [2]};
    int jj = ii+2;
    //std::cout << startcoords(0) << " " << startcoords(1) << " " << startcoords(2) << std::endl;
    bool LoS = true;
    while(LoS){
      if(jj >= path_info.details.path_length){
        break;
      }
      Eigen::Vector3d endcoords = {path_info.path[jj]->Data().transpose() [0], 
        path_info.path[jj]->Data().transpose() [1], path_info.path[jj]->Data().transpose() [2]};
      //std::cout << "end: " << endcoords(0) << " " << endcoords(1) << " " << endcoords(2) << std::endl;
      Eigen::Vector3d diff = endcoords - startcoords;
      int x = (int) abs(diff(0)), y = (int) abs(diff(1)), z = (int) abs(diff(2));
      //std::cout << x << " " << y << " " << z << std::endl;
      if(x >= y && x >= z){
        LoS = isVisible((int) startcoords(0), (int) startcoords(1), (int) startcoords(2), (int) endcoords(0), (int) endcoords(1), (int) endcoords(2), 0, &occupancy_grid);
      }
      else if(y >= x && y >= z){
        LoS = isVisible((int) startcoords(1), (int) startcoords(2), (int) startcoords(0), (int) endcoords(1), (int) endcoords(2), (int) endcoords(0), 1, &occupancy_grid);
      }
      else{
        LoS = isVisible((int) startcoords(2), (int) startcoords(0), (int) startcoords(1), (int) endcoords(2), (int) endcoords(0), (int) endcoords(1), 2, &occupancy_grid);
      }

      if(LoS){
        auto index = path_info.path.begin() + ii+1;
        path_info.path.erase(index);
        path_info.details.path_length = path_info.details.path_length-1;
      }
    }
  }

  //  path_info.details.Print(); //For debugging pruner

  std::cout << "=====  PATH   =====" << std::endl;
  for(const std::shared_ptr<Node3D>& node: path_info.path) {
    std::cout << "[" << node->Data().transpose() << "]" << std::endl;
  }

  std::cout << std::endl;

// set up p4 interpolation between waypoints
  std::chrono::duration<double> curr_dur = current_chrono_time - start_chrono_time;
  std::cout << "Current duration: " << curr_dur.count() << std::endl;
  //std::vector<double> times = {curr_dur.count()};
  //double prev_time = curr_dur.count();
  double prev_time = 0;
  std::vector<double> times = {0};

  std::cout << "=====  COORDS   =====" << std::endl;

  Eigen::Vector3d current_coords = occupancy_grid.boxCenter(current_pos_index(0), current_pos_index(1), current_pos_index(2));
    //Eigen::Vector3d current_coords = current_pos;

  std::vector<p4::NodeEqualityBound> node_equality_bounds = {
    // The parameter order for p4::NodeEqualityBound is:
    // (dimension_index, node_idx, derivative_idx, value)
    
    // The first node is the current position xyz
    p4::NodeEqualityBound(0,0,0,current_coords(0)),
    p4::NodeEqualityBound(1,0,0,current_coords(1)),
    p4::NodeEqualityBound(2,0,0,current_coords(2)),
    p4::NodeEqualityBound(0,0,1,0),
    p4::NodeEqualityBound(1,0,1,0),
    p4::NodeEqualityBound(2,0,1,0),
    p4::NodeEqualityBound(0,0,2,0),
    p4::NodeEqualityBound(1,0,2,0),
    p4::NodeEqualityBound(2,0,2,0),
  };
  std::cout << "[" << current_coords(0) << " " << current_coords(1) << " " << current_coords(2) << "]" << std::endl;


  //Feed the rest of the A* solution into the p4 interpolation
  //double index=0;
  Eigen::Vector3d prev_coords = current_coords;
  for(int ii = 1; ii<path_info.details.path_length; ii++) {
    Eigen::Vector3d coords = occupancy_grid.boxCenter(path_info.path[ii]->Data().transpose() [0], 
        path_info.path[ii]->Data().transpose() [1], path_info.path[ii]->Data().transpose() [2]);
    
    double x_val = coords(0);
    //std::cout << "X " << x_val << std::endl;
    double y_val = coords(1);
    //std::cout << "Y " << y_val << std::endl;
    double z_val = coords(2);
    //std::cout << "Z " << z_val << std::endl;
    
    //std::cout << "[" << x_val << " " << y_val << " " << z_val << "]" << std::endl;

    node_equality_bounds.push_back(p4::NodeEqualityBound(0,ii,0, x_val));
    node_equality_bounds.push_back(p4::NodeEqualityBound(1,ii,0, y_val));
    node_equality_bounds.push_back(p4::NodeEqualityBound(2,ii,0, z_val));

    double distance = (coords - prev_coords).norm();
    double time = (distance) / 1.9; //max vel is 2m/s, convert cell dist to m?
    double total_time = time + prev_time;
    times.push_back(total_time);


    prev_coords = coords;
    prev_time = total_time;
    //index++;
  }
    node_equality_bounds.push_back(p4::NodeEqualityBound(0,path_info.details.path_length-1,1, 0));
    node_equality_bounds.push_back(p4::NodeEqualityBound(1,path_info.details.path_length-1,1, 0));
    node_equality_bounds.push_back(p4::NodeEqualityBound(2,path_info.details.path_length-1,1, 0));
    node_equality_bounds.push_back(p4::NodeEqualityBound(0,path_info.details.path_length-1,2, 0));
    node_equality_bounds.push_back(p4::NodeEqualityBound(1,path_info.details.path_length-1,2, 0));
    node_equality_bounds.push_back(p4::NodeEqualityBound(2,path_info.details.path_length-1,2, 0));

  std::cout << std::endl;


  std::cout << "NEB Size: " << node_equality_bounds.size() << std::endl;

  std::cout << "=====  TIMES   =====" << std::endl;
  for(auto time : times) {
    std::cout << "[" << time << "]" << std::endl;
  }
  std::cout << std::endl;

  //segment inequality bounds to constrain velocity and acceleration
/*  std::vector<p4::SegmentInequalityBound> segment_inequality_bounds;
  for(int ii = 0; ii < (path_info.details.path_length-1); ii++){
    //segment_inequality_bounds.push_back(p4::SegmentInequalityBound(ii,1,Eigen::Vector3d(1,1,1),1.15));
    segment_inequality_bounds.push_back(p4::SegmentInequalityBound(ii,2,Eigen::Vector3d(1,0,0),0.4));
    segment_inequality_bounds.push_back(p4::SegmentInequalityBound(ii,2,Eigen::Vector3d(0,1,0),0.4));
    segment_inequality_bounds.push_back(p4::SegmentInequalityBound(ii,2,Eigen::Vector3d(0,0,1),0.4));
  }*/

  // Options to configure the polynomial solver with
  p4::PolynomialSolver::Options solver_options;
  solver_options.num_dimensions = 3;     // 3D
  solver_options.polynomial_order = 8;   // Fit an 8th-order polynomial
  solver_options.continuity_order = 4;   // Require continuity to the 4th order
  solver_options.derivative_order = 2;   // Minimize the 2rd order (acceleration)

  osqp_set_default_settings(&solver_options.osqp_settings);
  solver_options.osqp_settings.polish = true;       // Polish the solution, getting the best answer possible
  solver_options.osqp_settings.verbose = false;     // Suppress the printout

  // Use p4::PolynomialSolver object to solve for polynomial trajectories
  p4::PolynomialSolver solver(solver_options);
  p4::PolynomialSolver::Solution path
    = solver.Run(
        times, 
        node_equality_bounds, 
        {}, 
        {});

  // Sampling and Plotting
  // Plot 2D position
  // Options to configure the polynomial sampler with
  p4::PolynomialSampler::Options sampler_options;
  sampler_options.frequency = 100;             // Number of samples per second
  sampler_options.derivative_order = 0;        // Derivative to sample (0 = pos)

  // Use this object to sample a trajectory
  p4::PolynomialSampler sampler(sampler_options);
  Eigen::MatrixXd samples = sampler.Run(times, path);

  //Eigen::IOFormat cleanFmt(4, 0, ", ", "\n", "[", "]");
  //std::cout << samples.format(cleanFmt) << std::endl;

  sampler_options.derivative_order = 1;
  p4::PolynomialSampler samplerv(sampler_options);
  Eigen::MatrixXd samplesv = samplerv.Run(times, path);

  sampler_options.derivative_order = 2;
  p4::PolynomialSampler samplera(sampler_options);
  Eigen::MatrixXd samplesa = samplera.Run(times, path);


  //Feed into trajectory structure
  int NN = samples.cols();

  int id_curr = (int) ( curr_dur.count() / dt );

  current_chrono_time =
      std::chrono::system_clock::now();
  std::chrono::duration<double> begin_chrono = current_chrono_time.time_since_epoch();
  std::cout << "=========P4 Path===========" << std::endl;
  for (int idn = 0; idn < NN; idn++) {
    // chrono::duration<double> maintains high-precision floating point time in
    // seconds use the count function to cast into floating point

    //const std::chrono::duration<double> flight_chrono_time = begin_chrono + times[idn] * std::chrono::seconds(1);
    const std::chrono::duration<double> flight_chrono_time =
          begin_chrono + idn * dt_chrono; 

    // Calculate circular path points
    const double x = samples(1,idn);
    const double y = samples(2,idn);
    const double z = samples(3,idn);

    //std::cout << "[" << x << " " << y << " " << z << "]" << std::endl;

    // Generate velocities via chain rule
    const double vx = samplesv(1,idn);
    const double vy = samplesv(2,idn);
    const double vz = samplesv(3,idn);

    //std::cout << "[" << vx << " " << vy << " " << vz << "]" << std::endl;

    // Generate accelerations via chain rule
    const double ax = samplesa(1,idn);
    const double ay = samplesa(2,idn);
    const double az = samplesv(3,idn);

    //std::cout << "[" << ax << " " << ay << " " << az << "]" << std::endl;

    const double yaw = 0.0;

    // Time must be specified as a floating point number that measures the
    // number of seconds since the Unix epoch.
    const double flight_time = flight_chrono_time.count();

    // Push an a matrix packed with a trajectory point onto the trajectory
    // vector
    trajectory_vector.push_back((Eigen::Matrix<double, 11, 1>() << x, y, z, vx,
                                 vy, vz, ax, ay, az, yaw, flight_time)
                                    .finished());
    
  }
//  }
  trajectory_vector;
  // Construct a trajectory from the trajectory vector
  Trajectory trajectory(trajectory_vector);


  // Before submitting the trajectory, you can use this prevetting interface to
  // determine if the trajectory violates any mediation layer constraints. This
  // interface can be found in presubmission_trajectory_vetter.h. The PreVet
  // function returns type TrajectoryCode which contains three values: (1) The
  // mediation layer code that specifies success or the failure; (2) The failure
  // value (e.g., if the velocity limit is 4.0 m/s and you submit 5.0 m/s, the
  // value is returned as 5.0); (3) The failure index. This is the trajectory
  // sample index that caused the mediation layer to kick back an error code.
  // The index is the sampled index (specified from the P4 sampling process).
  // You can figure out the waypoint index by a simple math transformation.
  TrajectoryCode prevetter_response =
      prevetter_->PreVet(quad_name, trajectory, map3d_);
  switch (prevetter_response.code) {
    case MediationLayerCode::Success:
      // You probably won't need to do anything in response to Success.
      std::cout << "works!" << std::endl;
      break;
    /*case MediationLayerCode::TimeBetweenPointsExceedsMaxTime: {
      // Suppose your AP intends to submit a trajectory with a time that exceeds
      // the maximum allowed time between points. The prevetter would catch this
      // before you submit to the mediation layer.
      std::cout << "Prevet: Shorten time between trajectory points."
                << std::endl;
      std::cout << "Prevet: Time violation: " << prevetter_response.value
                << std::endl;
      break;
    }*/
    default:
      std::cout << "Prevet code: " << static_cast<int>(prevetter_response.code)
                << std::endl;
      std::cout << "Prevet value: " << prevetter_response.value << std::endl;
      std::cout << "Prevet index: " << prevetter_response.index << std::endl;
  }

  //=====================================================================================================
  //trajectory timing
  while (prevetter_response.code == MediationLayerCode::ExceedsMaxVelocity || 
      prevetter_response.code == MediationLayerCode::ExceedsMaxAcceleration ||
      prevetter_response.code == MediationLayerCode::MeanValueExceedsMaxAcceleration ||
      prevetter_response.code == MediationLayerCode::MeanValueExceedsMaxVelocity){

//    std::cout << "violation at prevetter index " << prevetter_response.index << " with value of " << prevetter_response.value << std::endl;

    // determine waypoint span where violation occurs (the waypoint past the violation)
    double bad_waypt_time = 1 / sampler_options.frequency * prevetter_response.index + times[0];
    int bad_waypt_index = 0;
    for (int ii = 0; ii < times.size(); ii++){
      if (prevetter_response.index == 0){
        bad_waypt_index = 0;
        break;
      }
      else if ((times[ii] - bad_waypt_time) >= 0){
        bad_waypt_index = ii;
        break;
      }
      else {
        continue;
      }
    }
    
//    std::cout << "adjusting times after waypoint index " << bad_waypt_index << " to ending index " << times.size() << std::endl;
    
    if (bad_waypt_index == 0){
      double relax = 0.1 * (times[bad_waypt_index]);
      for (int jj = bad_waypt_index; jj < times.size(); jj++){
        times[jj] = relax + times[jj];
      }
    }
    else {
      // relax time constraint along this span and adjust all spans after this index
      double relax = 0.2; //0.1 * (times[bad_waypt_index] - times[bad_waypt_index - 1]);
      for (int jj = bad_waypt_index; jj < times.size(); jj++){
        times[jj] = relax + times[jj];
      }
    }

//  std::cout << "===== ADJUSTED TIMES   =====" << std::endl;
//  for(auto time : times) {
//    std::cout << "[" << time << "]" << std::endl;
//  }
//  std::cout << std::endl;

    // Use p4::PolynomialSolver object to solve for polynomial trajectories
    p4::PolynomialSolver solver(solver_options);
    path = solver.Run(
        times, 
        node_equality_bounds, 
        {}, 
        {});

    // Use this object to sample a trajectory
    sampler_options.derivative_order = 0;
    p4::PolynomialSampler sampler(sampler_options);
    Eigen::MatrixXd samples = sampler.Run(times, path);

    sampler_options.derivative_order = 1;
    p4::PolynomialSampler samplerv(sampler_options);
    Eigen::MatrixXd samplesv = samplerv.Run(times, path);

    sampler_options.derivative_order = 2;
    p4::PolynomialSampler samplera(sampler_options);
    Eigen::MatrixXd samplesa = samplera.Run(times, path);

    //Feed into trajectory structure
    int NN = samples.cols();

    int id_curr = (int) ( curr_dur.count() / dt );

    trajectory_vector.clear();
    current_chrono_time =
      std::chrono::system_clock::now();
    begin_chrono = current_chrono_time.time_since_epoch();
    for (int idn = 0; idn < NN; idn++) {
      // chrono::duration<double> maintains high-precision floating point time in
      // seconds use the count function to cast into floating point

      //const std::chrono::duration<double> flight_chrono_time = begin_chrono + times[idn] * std::chrono::seconds(1);
      std::chrono::duration<double> flight_chrono_time =
          begin_chrono + idn * dt_chrono; 

      // Calculate circular path points
      const double x = samples(1,idn);
      const double y = samples(2,idn);
      const double z = samples(3,idn);

      // Generate velocities via chain rule
      const double vx = samplesv(1,idn);
      const double vy = samplesv(2,idn);
      const double vz = samplesv(3,idn);

      // Generate accelerations via chain rule
      const double ax = samplesa(1,idn);
      const double ay = samplesa(2,idn);
      const double az = samplesv(3,idn);

      const double yaw = 0.0;

      // Time must be specified as a floating point number that measures the
      // number of seconds since the Unix epoch.
      const double flight_time = flight_chrono_time.count();

      // Push an a matrix packed with a trajectory point onto the trajectory
      // vector
      trajectory_vector.push_back((Eigen::Matrix<double, 11, 1>() << x, y, z, vx,
                                 vy, vz, ax, ay, az, yaw, flight_time)
                                    .finished());
    }
    // Construct a trajectory from the trajectory vector
    Trajectory trajectory(trajectory_vector);

    prevetter_response =
      prevetter_->PreVet(quad_name, trajectory, map3d_);


  }
  switch (prevetter_response.code) {
    case MediationLayerCode::Success:
      // You probably won't need to do anything in response to Success.
      std::cout << "works!" << std::endl;
      break;
    /*case MediationLayerCode::TimeBetweenPointsExceedsMaxTime: {
      // Suppose your AP intends to submit a trajectory with a time that exceeds
      // the maximum allowed time between points. The prevetter would catch this
      // before you submit to the mediation layer.
      std::cout << "Prevet: Shorten time between trajectory points."
                << std::endl;
      std::cout << "Prevet: Time violation: " << prevetter_response.value
                << std::endl;
      break;
    }*/
    default:
      std::cout << "Prevet code: " << static_cast<int>(prevetter_response.code)
                << std::endl;
      std::cout << "Prevet value: " << prevetter_response.value << std::endl;
      std::cout << "Prevet index: " << prevetter_response.index << std::endl;
  }
  // Invoke the visualizer to see the proposed trajectory, which will be
  // displayed in violet. See student_game_engine_visualizer.h for other
  // visualization options: you can visualize a short path, a single point, etc.
  // It will be helpful to get such visual feedback on candidate trajectories.
  // Note that there is a built-in visualizer called "ViewManager" implemented
  // elsewhere in the game-engine code, but you don't have full control over
  // what it displays like you do with the Student_game_engine_visualizer
  // invoked below.
  Trajectory trajectoryadj(trajectory_vector);
  visualizer.drawTrajectory(trajectoryadj);
  quad_to_trajectory_map[quad_name] = trajectoryadj;
  return quad_to_trajectory_map;
}
}  // namespace game_engine








