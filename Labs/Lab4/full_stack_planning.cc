#include <cstdlib>
#include <vector>
#include <cmath>
#include <string>

#include "a_star2d.h"
#include "occupancy_grid2d.h"
#include "path_info.h"
#include "polynomial_solver.h"
#include "polynomial_sampler.h"
#include "gnuplot-iostream.h"
#include "gui2d.h"

using namespace game_engine;

/////////////////////////////////////////////////////////////////
//Function Prototypes
/////////////////////////////////////////////////////////////////
PathInfo RunAStar(
    const Graph2D& graph,
    const OccupancyGrid2D* occupancy_grid,
    const std::shared_ptr<Node2D>& start_ptr,
    const std::shared_ptr<Node2D>& end_ptr);

Eigen::MatrixXd PolyPlanner(const PathInfo path_info);

void writeToCSVfile(std::string name, Eigen::MatrixXd matrix);
/////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  if(argc != 6) {
    std::cerr << "Usage: ./full_stack_planning occupancy_grid_file row1 col1 row2 col2" << std::endl;
    return EXIT_FAILURE;
  }

  // Parsing input
  const std::string occupancy_grid_file = argv[1];
  const std::shared_ptr<Node2D> start_ptr = std::make_shared<Node2D>(
      Eigen::Vector2d(std::stoi(argv[2]),std::stoi(argv[3])));
  const std::shared_ptr<Node2D> end_ptr = std::make_shared<Node2D>(
      Eigen::Vector2d(std::stoi(argv[4]),std::stoi(argv[5])));

  // Load an occupancy grid from a file
  OccupancyGrid2D occupancy_grid;
  occupancy_grid.LoadFromFile(occupancy_grid_file);

  // Transform an occupancy grid into a graph
  const Graph2D graph = occupancy_grid.AsGraph();

  /////////////////////////////////////////////////////////////////////////////
  // RUN A STAR
  // TODO: Run your A* implementation over the graph and nodes defined above.
  //       This section is intended to be more free-form. Using previous
  //       problems and examples, determine the correct commands to complete
  //       this problem. You may want to take advantage of some of the plotting
  //       and graphing utilities in previous problems to check your solution on
  //       the way.
  /////////////////////////////////////////////////////////////////////////////

  PathInfo path_info = RunAStar(graph, &occupancy_grid, start_ptr, end_ptr);

  /////////////////////////////////////////////////////////////////////////////
  // RUN THE POLYNOMIAL PLANNER
  // TODO: Convert the A* solution to a problem the polynomial solver can
  //       solve. Solve the polynomial problem, sample the solution, figure out
  //       a way to export it to Matlab.
  /////////////////////////////////////////////////////////////////////////////

  Eigen::MatrixXd trajectory = PolyPlanner(path_info);

  writeToCSVfile("trajectory_info.csv",trajectory);

  return EXIT_SUCCESS;
}

//////////////////////////////////////////////////////////////////////////////
//Helper Functions
//////////////////////////////////////////////////////////////////////////////
PathInfo RunAStar(
    const Graph2D& graph,
    const OccupancyGrid2D* occupancy_grid,
    const std::shared_ptr<Node2D>& start_ptr,
    const std::shared_ptr<Node2D>& end_ptr) {
  
  std::cout << "============================================" << std::endl;
  std::cout << "=============  RUNNING A Star  =============" << std::endl;
  std::cout << "============================================" << std::endl;

  // Run A*
  AStar2D a_star;
  PathInfo path_info = a_star.Run(graph, start_ptr, end_ptr);

  // Display the solution
  Gui2D gui;
  gui.LoadOccupancyGrid(occupancy_grid);
  gui.LoadPath(path_info.path);
  gui.Display();

  // Print the solution
  path_info.details.Print();

  std::cout << "=====  PATH   =====" << std::endl;
  for(const std::shared_ptr<Node2D>& node: path_info.path) {
    std::cout << "[" << node->Data().transpose() << "]" << std::endl;
  }

  std::cout << std::endl;
  return path_info;

}

Eigen::MatrixXd PolyPlanner(const PathInfo path_info) {
  // Time in seconds
  auto path_size = path_info.details.path_length;
  std::vector<double> times = {};
  for (auto n = 0; n<path_size; n++){
    times.push_back(n);
  } 

  // The parameter order for p4::NodeEqualityBound is:
  // (dimension_index, node_idx, derivative_idx, value)
  std::vector<p4::NodeEqualityBound> node_equality_bounds = {
    // The first node must constrain position, velocity, and acceleration
    p4::NodeEqualityBound(0,0,0,path_info.path[0]->Data().x()),
    p4::NodeEqualityBound(1,0,0,path_info.path[0]->Data().y()),
    p4::NodeEqualityBound(0,0,1,path_info.path[0]->Data().x()),
    p4::NodeEqualityBound(1,0,1,path_info.path[0]->Data().y()),
    p4::NodeEqualityBound(0,0,2,path_info.path[0]->Data().x()),
    p4::NodeEqualityBound(1,0,2,path_info.path[0]->Data().y()),
  };
    
  for (auto n = 1; n<(path_size-1); n++){
    auto xpos = path_info.path[n]->Data().x();
    auto ypos = path_info.path[n]->Data().y();
    node_equality_bounds.push_back(p4::NodeEqualityBound(0,n,0,xpos));
    node_equality_bounds.push_back(p4::NodeEqualityBound(1,n,0,ypos));
    
    /*
    std::cout<< "n = " << n <<std::endl;
    std::cout<< "time = " << times[n] <<std::endl;
    std::cout<< "xpos = " << xpos <<std::endl;
    std::cout<< "ypos = " << ypos <<std::endl;
    */
  }

  // The final node constrains position
  node_equality_bounds.push_back(p4::NodeEqualityBound(0,path_size-1,0,path_info.path[path_size-1]->Data().x()));
  node_equality_bounds.push_back(p4::NodeEqualityBound(1,path_size-1,0,path_info.path[path_size-1]->Data().y()));
  
  // std::cout<< node_equality_bounds.size()<<std::endl;

  // Options to configure the polynomial solver with
  p4::PolynomialSolver::Options solver_options;
  solver_options.num_dimensions = 2;     // 2D
  solver_options.polynomial_order = 8;   // Fit an 8th-order polynomial
  solver_options.continuity_order = 4;   // Require continuity to the 4th order
  solver_options.derivative_order = 2;   // Minimize the 2nd order (acceleration)

  osqp_set_default_settings(&solver_options.osqp_settings);
  solver_options.osqp_settings.polish = true;       // Polish the solution, getting the best answer possible
  solver_options.osqp_settings.verbose = false;     // Suppress the printout

  // Use p4::PolynomialSolver object to solve for polynomial trajectories
  p4::PolynomialSolver solver(solver_options);
  const p4::PolynomialSolver::Solution path
    = solver.Run(
        times, 
        node_equality_bounds, 
        {}, 
        {});

  // Sampling and Plotting
  { // Plot 2D position
    // Options to configure the polynomial sampler with
    p4::PolynomialSampler::Options sampler_options;
    sampler_options.frequency = 200;             // Number of samples per second
    sampler_options.derivative_order = 0;        // Derivative to sample (0 = pos)

    // Use this object to sample a trajectory
    p4::PolynomialSampler sampler(sampler_options);
    Eigen::MatrixXd samples = sampler.Run(times, path);

    // Plotting tool requires vectors
    std::vector<double> t_hist, x_hist, y_hist;
    for(size_t time_idx = 0; time_idx < samples.cols(); ++time_idx) {
      t_hist.push_back(samples(0,time_idx));
      x_hist.push_back(samples(1,time_idx));
      y_hist.push_back(samples(2,time_idx));
    }

    // gnu-iostream plotting library
    // Utilizes gnuplot commands with a nice stream interface
    {
      Gnuplot gp;
      gp << "plot '-' using 1:2 with lines title 'Trajectory'" << std::endl;
      gp.send1d(boost::make_tuple(x_hist, y_hist));
      gp << "set grid" << std::endl;
      gp << "set xlabel 'X'" << std::endl;
      gp << "set ylabel 'Y'" << std::endl;
      gp << "replot" << std::endl;
    }
    {
      Gnuplot gp;
      gp << "plot '-' using 1:2 with lines title 'X-Profile'" << std::endl;
      gp.send1d(boost::make_tuple(t_hist, x_hist));
      gp << "set grid" << std::endl;
      gp << "set xlabel 'Time (s)'" << std::endl;
      gp << "set ylabel 'X-Profile'" << std::endl;
      gp << "replot" << std::endl;
    }
    {
      Gnuplot gp;
      gp << "plot '-' using 1:2 with lines title 'Y-Profile'" << std::endl;
      gp.send1d(boost::make_tuple(t_hist, y_hist));
      gp << "set grid" << std::endl;
      gp << "set xlabel 'Time (s)'" << std::endl;
      gp << "set ylabel 'Y-Profile'" << std::endl;
      gp << "replot" << std::endl;
    }

  // Velocity
    // Options to configure the polynomial sampler with
    p4::PolynomialSampler::Options sampler_options_v;
    sampler_options_v.frequency = 200;             // Number of samples per second
    sampler_options_v.derivative_order = 1;        // Derivative to sample (1 = vel)

    // Use this object to sample a trajectory
    p4::PolynomialSampler sampler_v(sampler_options_v);
    Eigen::MatrixXd samples_v = sampler_v.Run(times, path);

    // Plotting tool requires vectors
    std::vector<double> xdot_hist, ydot_hist;
    for(size_t time_idx = 0; time_idx < samples_v.cols(); ++time_idx) {
      xdot_hist.push_back(samples_v(1,time_idx));
      ydot_hist.push_back(samples_v(2,time_idx));
    }

  // Acceleration
    // Options to configure the polynomial sampler with
    p4::PolynomialSampler::Options sampler_options_a;
    sampler_options_a.frequency = 200;             // Number of samples per second
    sampler_options_a.derivative_order = 2;        // Derivative to sample (2 = acc)

    // Use this object to sample a trajectory
    p4::PolynomialSampler sampler_a(sampler_options_a);
    Eigen::MatrixXd samples_a = sampler_a.Run(times, path);

    // Plotting tool requires vectors
    std::vector<double> xdotdot_hist, ydotdot_hist;
    for(size_t time_idx = 0; time_idx < samples.cols(); ++time_idx) {
      xdotdot_hist.push_back(samples_a(1,time_idx));
      ydotdot_hist.push_back(samples_a(2,time_idx));
    }

  //Matrix of all histories
    Eigen::MatrixXd time = Eigen::Map<Eigen::MatrixXd>(t_hist.data(),t_hist.size(),1);
    Eigen::MatrixXd xpos = Eigen::Map<Eigen::MatrixXd>(x_hist.data(),t_hist.size(),1);
    Eigen::MatrixXd ypos = Eigen::Map<Eigen::MatrixXd>(y_hist.data(),t_hist.size(),1);
    Eigen::MatrixXd xvel = Eigen::Map<Eigen::MatrixXd>(xdot_hist.data(),t_hist.size(),1);
    Eigen::MatrixXd yvel = Eigen::Map<Eigen::MatrixXd>(ydot_hist.data(),t_hist.size(),1);
    Eigen::MatrixXd xacc = Eigen::Map<Eigen::MatrixXd>(xdotdot_hist.data(),t_hist.size(),1);
    Eigen::MatrixXd yacc = Eigen::Map<Eigen::MatrixXd>(ydotdot_hist.data(),t_hist.size(),1);

    Eigen::MatrixXd trajectory(time.rows(),time.cols()+xpos.cols()+ypos.cols()+xvel.cols()+yvel.cols()+xacc.cols()+yacc.cols());
    trajectory << time, xpos, ypos, xvel, yvel, xacc, yacc;
    return trajectory;
  }

  //return trajectory;

}


const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");
void writeToCSVfile(std::string name, Eigen::MatrixXd matrix){
  std::ofstream file(name.c_str());
  file << matrix.format(CSVFormat);
} 
