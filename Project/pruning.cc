#include "pruning.h"

namespace game_engine {
    bool isVisible(int longi, int short1i, int short2i, int longf, int short1f, int short2f, int longdim, OccupancyGrid3D* occupancy_grid){
    	// Returns true if line of sight from cell i to cell f is visible (traversible by quad).
    	// Computations done via Bresenham's algorithm applied to a 3D grid.
    	// The inputs correspond to 3D cell indices, the long index dimension, and the occupancy grid to be checked. We assume
    	// the first index dimension input is the greatest (or tied for the greatest) dimensional difference between the two cells
    	// for the purpose of calculating slopes. Then, the short1 and short2 index dimensions correspond to the next dimensions
    	// in looping xyz convention, i.e. starting at longdim = 1 (y dimension), short1 is z component and short2 is x component
    	double slope1 = double(short1f-short1i)/double(longf-longi);
    	double slope2 = double(short2f-short2i)/double(longf-longi);
    	int inc = 1;
    	if (longf < longi){
    		inc = -1;
    	}
    	for (int currlong = longi; currlong != longf+inc; currlong+=inc){
    		int currshort1 = round(slope1*(currlong-longi)+short1i);
    		int currshort2 = round(slope2*(currlong-longi)+short2i);
    		switch(longdim) {
    			case 0:
    				if (occupancy_grid->IsOccupied(currshort2, currshort1, currlong)){
    					return false;
    				}
    				break;
    			case 1:
    				if (occupancy_grid->IsOccupied(currshort1, currlong, currshort2)){
    					return false;
    				}
    				break;
    			case 2:
    				if (occupancy_grid->IsOccupied(currlong, currshort2, currshort1)){
    					return false;
    				}
    				break;
    		}
    	}
    	return true;
    }
}
