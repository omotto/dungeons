// A C++ Program to implement A* Search Algorithm
#include <iostream>
#include <vector> 			// vector
#include <cfloat>			//FLT_MAX definition
#include <utility>  		// make_pair template
#include <stack> 			// stack template
#include <set> 				// set template
#include <math.h> 			//sqrt function

#include "SDL.h"
#include "SDL_image.h"
#include "SDL_ttf.h"
#include "SDL_mixer.h"

typedef std::pair<int, int> Pair; 						// Creating a shortcut for int, int pair type
typedef std::pair<double, std::pair<int, int> > pPair; 	// Creating a shortcut for pair<int, pair<int, int>> type

struct cell {											// A structure to hold the neccesary parameters
    int parent_i, parent_j; 							// Row and Column index of its parent (Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1)
    double f, g, h;     								// f = g + h
};

// A Utility Function to check whether given cell (row, col) is a valid cell or not.
bool isValid(int row, int col, int rows, int cols) {
    return (row >= 0) && (row < rows) && (col >= 0) && (col < cols); // Returns true if row number and column number is in range
}

// A Utility Function to check whether destination cell has been reached or not
bool isDestination(int row, int col, Pair dest) {
    if (row == dest.first && col == dest.second) return (true); else return (false);
}

// A Utility Function to calculate the 'h' heuristics.
double calculateHValue(int row, int col, Pair dest) {
    return ((double)sqrt ((double)(row-dest.first)*(row-dest.first) + (col-dest.second)*(col-dest.second)));
    //return (double)(fabs(row-dest.first) + fabs(col-dest.second));
}

// A Function to find the shortest path between a given source cell to a destination cell according to A* Search Algorithm
bool aStarSearch(std::vector< std::vector<int> > grid, int rows, int cols, Pair src, Pair dest, std::stack<Pair> & Path) {
    // If the source is out of range
    if (isValid (src.first, src.second, rows, cols) == false) { /*SDL_Log("Source is invalid\r\n");*/ return false; }
    // If the destination is out of range
    if (isValid (dest.first, dest.second, rows, cols) == false) { /*SDL_Log("Destination is invalid\r\n");*/ return false; }
    // Either the source or the destination is blocked
    if (grid[src.first][src.second] == 0 || grid[dest.first][dest.second] == 0) { /*SDL_Log("Source or the destination is blocked\r\n");*/ return false; }
    // If the destination cell is the same as source cell
    if (isDestination(src.first, src.second, dest) == true) { /*SDL_Log("We are already at the destination\r\n");*/ return false; }
    // Create a closed list and initialise it to false which means that no cell has been included yet
    std::vector< std::vector<bool> > closedList(rows, std::vector<bool>(cols));
    // Declare a 2D array of structure to hold the details of that cell
    std::vector< std::vector<cell> > cellDetails(rows, std::vector<cell>(cols));
    // --
    int i, j;
    //
    for (i = 0; i < rows; i++) {
        for (j = 0; j < cols; j++) {
            cellDetails[i][j].f = FLT_MAX;
            cellDetails[i][j].g = FLT_MAX;
            cellDetails[i][j].h = FLT_MAX;
            cellDetails[i][j].parent_i = -1;
            cellDetails[i][j].parent_j = -1;
            closedList[i][j] = false;
        }
    }
    // Initialising the parameters of the starting node
    i = src.first, j = src.second;
    cellDetails[i][j].f = 0.0;
    cellDetails[i][j].g = 0.0;
    cellDetails[i][j].h = 0.0;
    cellDetails[i][j].parent_i = i;
    cellDetails[i][j].parent_j = j;
    // Create an open list having information as <f, <i, j>> where f = g + h, and i, j are the row and column index of that cell
    // Note that 0 <= i <= rows-1 & 0 <= j <= cols-1 This open list is implenented as a set of pair of pair.
    std::set<pPair> openList;
    // Put the starting cell on the open list and set its 'f' as 0
    openList.insert(std::make_pair (0.0, std::make_pair(i, j)));
    while (!openList.empty()) {
        pPair p = *openList.begin();
        // Remove this vertex from the open list
        openList.erase(openList.begin());
        // Add this vertex to the open list
        i = p.second.first;
        j = p.second.second;
        closedList[i][j] = true;
        /*
         Generating all the 8 successor of this cell

             N.W   N   N.E
               \   |   /
                \  |  /
             W----Cell----E
                  / | \
                /   |  \
             S.W    S   S.E

         Cell-->Popped Cell (i, j)
         N -->  North       (i-1, j)
         S -->  South       (i+1, j)
         E -->  East        (i, j+1)
         W -->  West           (i, j-1)
         N.E--> North-East  (i-1, j+1)
         N.W--> North-West  (i-1, j-1)
         S.E--> South-East  (i+1, j+1)
         S.W--> South-West  (i+1, j-1)*/
        // To store the 'g', 'h' and 'f' of the 8 successors
        double gNew, hNew, fNew;
        //----------- 1st Successor (North) ------------
        // Only process this cell if this is a valid one
        if (isValid(i-1, j, rows, cols) == true) {
            // If the destination cell is the same as the current successor
            if (isDestination(i-1, j, dest) == true) {
                // Set the Parent of the destination cell
                cellDetails[i-1][j].parent_i = i;
                cellDetails[i-1][j].parent_j = j;
                // path found
                int row = dest.first;
                int col = dest.second;
                while (!(cellDetails[row][col].parent_i == row && cellDetails[row][col].parent_j == col )) {
                    Path.push (std::make_pair (row, col));
                    int temp_row = cellDetails[row][col].parent_i;
                    int temp_col = cellDetails[row][col].parent_j;
                    row = temp_row;
                    col = temp_col;
                }
                Path.push (std::make_pair (row, col));
                return true;
            }
                // If the successor is already on the closed list or if it is blocked, then ignore it. Else do the following
            else if (closedList[i-1][j] == false && grid[i-1][j] == 1) {
                gNew = cellDetails[i][j].g + 1.0;
                hNew = calculateHValue (i-1, j, dest);
                fNew = gNew + hNew;
                // If it isn?t on the open list, add it to the open list. Make the current square the parent of this square. Record the f, g, and h costs of the square cell
                //                OR
                // If it is on the open list already, check to see if this path to that square is better, using 'f' cost as the measure.
                if (cellDetails[i-1][j].f == FLT_MAX || cellDetails[i-1][j].f > fNew) {
                    openList.insert(std::make_pair(fNew, std::make_pair(i-1, j)));
                    // Update the details of this cell
                    cellDetails[i-1][j].f = fNew;
                    cellDetails[i-1][j].g = gNew;
                    cellDetails[i-1][j].h = hNew;
                    cellDetails[i-1][j].parent_i = i;
                    cellDetails[i-1][j].parent_j = j;
                }
            }
        }
        //----------- 2nd Successor (South) ------------
        // Only process this cell if this is a valid one
        if (isValid(i+1, j, rows, cols) == true) {
            // If the destination cell is the same as the current successor
            if (isDestination(i+1, j, dest) == true) {
                // Set the Parent of the destination cell
                cellDetails[i+1][j].parent_i = i;
                cellDetails[i+1][j].parent_j = j;
                // path found
                int row = dest.first;
                int col = dest.second;
                while (!(cellDetails[row][col].parent_i == row && cellDetails[row][col].parent_j == col )) {
                    Path.push (std::make_pair (row, col));
                    int temp_row = cellDetails[row][col].parent_i;
                    int temp_col = cellDetails[row][col].parent_j;
                    row = temp_row;
                    col = temp_col;
                }
                Path.push (std::make_pair (row, col));
                return true;
            }
                // If the successor is already on the closed list or if it is blocked, then ignore it. Else do the following
            else if (closedList[i+1][j] == false && grid[i+1][j] == 1) {
                gNew = cellDetails[i][j].g + 1.0;
                hNew = calculateHValue(i+1, j, dest);
                fNew = gNew + hNew;
                // If it isn?t on the open list, add it to the open list. Make the current square the parent of this square. Record the f, g, and h costs of the square cell
                //                OR
                // If it is on the open list already, check to see if this path to that square is better, using 'f' cost as the measure.
                if (cellDetails[i+1][j].f == FLT_MAX || cellDetails[i+1][j].f > fNew) {
                    openList.insert(std::make_pair (fNew, std::make_pair(i+1, j)));
                    // Update the details of this cell
                    cellDetails[i+1][j].f = fNew;
                    cellDetails[i+1][j].g = gNew;
                    cellDetails[i+1][j].h = hNew;
                    cellDetails[i+1][j].parent_i = i;
                    cellDetails[i+1][j].parent_j = j;
                }
            }
        }
        //----------- 3rd Successor (East) ------------
        // Only process this cell if this is a valid one
        if (isValid (i, j+1, rows, cols) == true) {
            // If the destination cell is the same as the current successor
            if (isDestination(i, j+1, dest) == true) {
                // Set the Parent of the destination cell
                cellDetails[i][j+1].parent_i = i;
                cellDetails[i][j+1].parent_j = j;
                // path found
                int row = dest.first;
                int col = dest.second;
                while (!(cellDetails[row][col].parent_i == row && cellDetails[row][col].parent_j == col )) {
                    Path.push (std::make_pair (row, col));
                    int temp_row = cellDetails[row][col].parent_i;
                    int temp_col = cellDetails[row][col].parent_j;
                    row = temp_row;
                    col = temp_col;
                }
                Path.push (std::make_pair (row, col));
                return true;
            }
                // If the successor is already on the closed list or if it is blocked, then ignore it. Else do the following
            else if (closedList[i][j+1] == false && grid[i][j+1] == 1) {
                gNew = cellDetails[i][j].g + 1.0;
                hNew = calculateHValue (i, j+1, dest);
                fNew = gNew + hNew;
                // If it isn?t on the open list, add it to the open list. Make the current square the parent of this square. Record the f, g, and h costs of the square cell
                //                OR
                // If it is on the open list already, check to see if this path to that square is better, using 'f' cost as the measure.
                if (cellDetails[i][j+1].f == FLT_MAX || cellDetails[i][j+1].f > fNew) {
                    openList.insert(std::make_pair(fNew, std::make_pair(i, j+1)));
                    // Update the details of this cell
                    cellDetails[i][j+1].f = fNew;
                    cellDetails[i][j+1].g = gNew;
                    cellDetails[i][j+1].h = hNew;
                    cellDetails[i][j+1].parent_i = i;
                    cellDetails[i][j+1].parent_j = j;
                }
            }
        }
        //----------- 4th Successor (West) ------------
        // Only process this cell if this is a valid one
        if (isValid(i, j-1, rows, cols) == true) {
            // If the destination cell is the same as the current successor
            if (isDestination(i, j-1, dest) == true) {
                // Set the Parent of the destination cell
                cellDetails[i][j-1].parent_i = i;
                cellDetails[i][j-1].parent_j = j;
                // path found
                int row = dest.first;
                int col = dest.second;
                while (!(cellDetails[row][col].parent_i == row && cellDetails[row][col].parent_j == col )) {
                    Path.push (std::make_pair (row, col));
                    int temp_row = cellDetails[row][col].parent_i;
                    int temp_col = cellDetails[row][col].parent_j;
                    row = temp_row;
                    col = temp_col;
                }
                Path.push (std::make_pair (row, col));
                return true;
            }
                // If the successor is already on the closed list or if it is blocked, then ignore it. Else do the following
            else if (closedList[i][j-1] == false && grid[i][j-1] == 1) {
                gNew = cellDetails[i][j].g + 1.0;
                hNew = calculateHValue(i, j-1, dest);
                fNew = gNew + hNew;
                // If it isn?t on the open list, add it to the open list. Make the current square the parent of this square. Record the f, g, and h costs of the square cell
                //                OR
                // If it is on the open list already, check to see if this path to that square is better, using 'f' cost as the measure.
                if (cellDetails[i][j-1].f == FLT_MAX || cellDetails[i][j-1].f > fNew) {
                    openList.insert(std::make_pair(fNew, std::make_pair(i, j-1)));
                    // Update the details of this cell
                    cellDetails[i][j-1].f = fNew;
                    cellDetails[i][j-1].g = gNew;
                    cellDetails[i][j-1].h = hNew;
                    cellDetails[i][j-1].parent_i = i;
                    cellDetails[i][j-1].parent_j = j;
                }
            }
        }
        //----------- 5th Successor (North-East) ------------
        // Only process this cell if this is a valid one
        if (isValid(i-1, j+1, rows, cols) == true) {
            // If the destination cell is the same as the current successor
            if (isDestination(i-1, j+1, dest) == true) {
                // Set the Parent of the destination cell
                cellDetails[i-1][j+1].parent_i = i;
                cellDetails[i-1][j+1].parent_j = j;
                // path found
                int row = dest.first;
                int col = dest.second;
                while (!(cellDetails[row][col].parent_i == row && cellDetails[row][col].parent_j == col )) {
                    Path.push (std::make_pair (row, col));
                    int temp_row = cellDetails[row][col].parent_i;
                    int temp_col = cellDetails[row][col].parent_j;
                    row = temp_row;
                    col = temp_col;
                }
                Path.push (std::make_pair (row, col));
                return true;
            }
                // If the successor is already on the closed list or if it is blocked, then ignore it. Else do the following
            else if (closedList[i-1][j+1] == false && grid[i-1][j+1] == 1) {
                gNew = cellDetails[i][j].g + 1.414;
                hNew = calculateHValue(i-1, j+1, dest);
                fNew = gNew + hNew;
                // If it isn?t on the open list, add it to the open list. Make the current square the parent of this square. Record the f, g, and h costs of the square cell
                //                OR
                // If it is on the open list already, check to see if this path to that square is better, using 'f' cost as the measure.
                if (cellDetails[i-1][j+1].f == FLT_MAX || cellDetails[i-1][j+1].f > fNew) {
                    openList.insert( make_pair (fNew, std::make_pair(i-1, j+1)));
                    // Update the details of this cell
                    cellDetails[i-1][j+1].f = fNew;
                    cellDetails[i-1][j+1].g = gNew;
                    cellDetails[i-1][j+1].h = hNew;
                    cellDetails[i-1][j+1].parent_i = i;
                    cellDetails[i-1][j+1].parent_j = j;
                }
            }
        }
        //----------- 6th Successor (North-West) ------------
        // Only process this cell if this is a valid one
        if (isValid (i-1, j-1, rows, cols) == true) {
            // If the destination cell is the same as the current successor
            if (isDestination (i-1, j-1, dest) == true) {
                // Set the Parent of the destination cell
                cellDetails[i-1][j-1].parent_i = i;
                cellDetails[i-1][j-1].parent_j = j;
                // path found
                int row = dest.first;
                int col = dest.second;
                while (!(cellDetails[row][col].parent_i == row && cellDetails[row][col].parent_j == col )) {
                    Path.push (std::make_pair (row, col));
                    int temp_row = cellDetails[row][col].parent_i;
                    int temp_col = cellDetails[row][col].parent_j;
                    row = temp_row;
                    col = temp_col;
                }
                Path.push (std::make_pair (row, col));
                return true;
            }
                // If the successor is already on the closed list or if it is blocked, then ignore it. Else do the following
            else if (closedList[i-1][j-1] == false && grid[i-1][j-1] == 1) {
                gNew = cellDetails[i][j].g + 1.414;
                hNew = calculateHValue(i-1, j-1, dest);
                fNew = gNew + hNew;
                // If it isn?t on the open list, add it to the open list. Make the current square the parent of this square. Record the f, g, and h costs of the square cell
                //                OR
                // If it is on the open list already, check to see if this path to that square is better, using 'f' cost as the measure.
                if (cellDetails[i-1][j-1].f == FLT_MAX || cellDetails[i-1][j-1].f > fNew) {
                    openList.insert( make_pair (fNew, std::make_pair(i-1, j-1)));
                    // Update the details of this cell
                    cellDetails[i-1][j-1].f = fNew;
                    cellDetails[i-1][j-1].g = gNew;
                    cellDetails[i-1][j-1].h = hNew;
                    cellDetails[i-1][j-1].parent_i = i;
                    cellDetails[i-1][j-1].parent_j = j;
                }
            }
        }
        //----------- 7th Successor (South-East) ------------
        // Only process this cell if this is a valid one
        if (isValid(i+1, j+1, rows, cols) == true) {
            // If the destination cell is the same as the current successor
            if (isDestination(i+1, j+1, dest) == true) {
                // Set the Parent of the destination cell
                cellDetails[i+1][j+1].parent_i = i;
                cellDetails[i+1][j+1].parent_j = j;
                // path found
                int row = dest.first;
                int col = dest.second;
                while (!(cellDetails[row][col].parent_i == row && cellDetails[row][col].parent_j == col )) {
                    Path.push (std::make_pair (row, col));
                    int temp_row = cellDetails[row][col].parent_i;
                    int temp_col = cellDetails[row][col].parent_j;
                    row = temp_row;
                    col = temp_col;
                }
                Path.push (std::make_pair (row, col));
                return true;
            }
                // If the successor is already on the closed list or if it is blocked, then ignore it. Else do the following
            else if (closedList[i+1][j+1] == false && grid[i+1][j+1] == 1) {
                gNew = cellDetails[i][j].g + 1.414;
                hNew = calculateHValue(i+1, j+1, dest);
                fNew = gNew + hNew;
                // If it isn?t on the open list, add it to the open list. Make the current square the parent of this square. Record the f, g, and h costs of the square cell
                //                OR
                // If it is on the open list already, check to see if this path to that square is better, using 'f' cost as the measure.
                if (cellDetails[i+1][j+1].f == FLT_MAX || cellDetails[i+1][j+1].f > fNew) {
                    openList.insert(make_pair(fNew, std::make_pair(i+1, j+1)));
                    // Update the details of this cell
                    cellDetails[i+1][j+1].f = fNew;
                    cellDetails[i+1][j+1].g = gNew;
                    cellDetails[i+1][j+1].h = hNew;
                    cellDetails[i+1][j+1].parent_i = i;
                    cellDetails[i+1][j+1].parent_j = j;
                }
            }
        }
        //----------- 8th Successor (South-West) ------------
        // Only process this cell if this is a valid one
        if (isValid (i+1, j-1, rows, cols) == true) {
            // If the destination cell is the same as the current successor
            if (isDestination(i+1, j-1, dest) == true) {
                // Set the Parent of the destination cell
                cellDetails[i+1][j-1].parent_i = i;
                cellDetails[i+1][j-1].parent_j = j;
                // path found
                int row = dest.first;
                int col = dest.second;
                while (!(cellDetails[row][col].parent_i == row && cellDetails[row][col].parent_j == col )) {
                    Path.push (std::make_pair (row, col));
                    int temp_row = cellDetails[row][col].parent_i;
                    int temp_col = cellDetails[row][col].parent_j;
                    row = temp_row;
                    col = temp_col;
                }
                Path.push (std::make_pair (row, col));
                return true;
            }
                // If the successor is already on the closed list or if it is blocked, then ignore it. Else do the following
            else if (closedList[i+1][j-1] == false && grid[i+1][j-1] == 1) {
                gNew = cellDetails[i][j].g + 1.414;
                hNew = calculateHValue(i+1, j-1, dest);
                fNew = gNew + hNew;
                // If it isn?t on the open list, add it to the open list. Make the current square the parent of this square. Record the f, g, and h costs of the square cell
                //                OR
                // If it is on the open list already, check to see if this path to that square is better, using 'f' cost as the measure.
                if (cellDetails[i+1][j-1].f == FLT_MAX || cellDetails[i+1][j-1].f > fNew) {
                    openList.insert(make_pair(fNew, std::make_pair(i+1, j-1)));
                    // Update the details of this cell
                    cellDetails[i+1][j-1].f = fNew;
                    cellDetails[i+1][j-1].g = gNew;
                    cellDetails[i+1][j-1].h = hNew;
                    cellDetails[i+1][j-1].parent_i = i;
                    cellDetails[i+1][j-1].parent_j = j;
                }
            }
        }
    }
    return false;
}

// ------------------------------------------------------------------------------------------------

#include <stdlib.h>     // srand, rand 
#include <time.h>       // time 

class Shake {
public:
	Shake(int duration, int frequency) {
		this->duration = duration; 		// The duration in milliseconds
		this->frequency = frequency; 	// The frequency in Hz
		int sampleCount = (duration/1000) * frequency;  // The sample count = number of peaks/valleys in the Shake
	 	// Populate the samples array with randomized values between -1.0 and 1.0
 		for (int i = 0; i < sampleCount; i++)
			this->samples.push_back(((float) rand() / (RAND_MAX)) * 2.0 - 1.0);
     	// Init the time variables
    	this->startTime = 0;
    	this->t         = 0;
    	// Flag that represents if the shake is active
 		this->isShaking = false;
	}
	~Shake() {
		this->samples.clear();
	}
	void set(int duration, int frequency) {
		this->duration = duration;
		this->frequency = frequency;
	}
	void start(void) {
		this->startTime = SDL_GetTicks();
		this->t         = 0;
		this->isShaking = true;
	}
	float update(void) {
		this->t = SDL_GetTicks() - this->startTime;
		if (this->t > this->duration)  this->isShaking = false;
		// Get the previous and next sample
		float k = (this->t) / 1000.0 * this->frequency;
		int   i = floor(k),
			  j = i + 1;
		// Return the current amplitide
		return (this->noise(i) + (float)(k - i)*(this->noise(j) - this->noise(i))) * this->decay(this->t);		
	}
	float noise(int sampleNum) {
		if (sampleNum >= this->samples.size()) return 0;
		return this->samples[sampleNum];
	}
	float decay(int time) {
		if (time >= this->duration) return 0;
		return (float)(this->duration - time) / (float)this->duration;
	}
	bool getIsShaking(void)	{
		return this->isShaking;
	}
private:
	std::vector<float> samples;
	int  duration, frequency;
	int  t, startTime;
	bool isShaking;
};

// ------------------------------------------------------------------------------------------------

const SDL_Color WHITE_COLOR 		= {.r = 0xFF, .g = 0xFF, .b = 0xFF, .a = 0xFF};		
const SDL_Color BLACK_COLOR 		= {.r = 0x00, .g = 0x00, .b = 0x00, .a = 0xFF};	

class Fade {
public:
	Fade(SDL_Color color, bool inout, int duration) {
		this->color 	= color;
		this->finished 	= true;
		this->inout 	= inout;
		this->step		= duration / 255;
	}
	~Fade(){ }
	void set(SDL_Color color, bool inout, int duration) {
		this->color 	= color;
		this->inout 	= inout;
		this->step		= duration / 255;
	}
	void start(void) {
		this->ms		= SDL_GetTicks();
		this->finished 	= false;
		if (this->inout == true) this->level = 0; else this->level = 255;
	}
	void update(SDL_Renderer* renderer, int screenWidth, int screenHeight) {
		if (finished == false) {
			if ((SDL_GetTicks() - this->ms) > this->step) {
				SDL_Rect rect;
				rect.x = 0;
				rect.y = 0;
				rect.w = screenWidth;
				rect.h = screenHeight;
				// --		
				if (this->inout == true) {
					if (this->level < 255) this->level++; else finished = true;
				} else {
					if (this->level > 0) this->level--; else finished = true;
				}
				// --
				SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND); // To allow alpha blending
				SDL_SetRenderDrawColor(renderer, this->color.r, this->color.g, this->color.b, this->level);
				SDL_RenderFillRect(renderer, & rect);		
				this->ms = SDL_GetTicks();
			}
		}
	}
	bool isFinished(void) {
		return finished;
	}
private:	
	bool 		finished; 	// Fade effect is finished?	
	SDL_Color 	color;		// Initial color
	uint8_t 	level;		// Current fade level [0..255]
	int 		step; 		// Step in ms 
	int			ms;			// Time in ms
	bool 		inout;		// Fade on or Fade off?
	
};

// ------------------------------------------------------------------------------------------------

int findNearMultipleof(int number, int multiple) {
	int prevDistance = 0xFFFFFF, counter = 1;
	while(1) {
		if (abs(number - multiple*counter) > prevDistance) return multiple*(counter-1);
		prevDistance = abs(number - multiple*counter);
		counter++;
	}
}

SDL_Texture * loadSpriteSheet(SDL_Renderer* renderer, const char * filename) {
    SDL_Surface * spriteSheet = IMG_Load(filename);
    SDL_Texture * texture 	  = SDL_CreateTextureFromSurface(renderer, spriteSheet);
    SDL_FreeSurface(spriteSheet);
    return texture;
}

bool checkCollision (SDL_Rect a, SDL_Rect b, int offsetX = 0, int offsetY = 0) {
    int leftA  	= a.x,
        leftB  	= b.x,
        rightA 	= a.x + a.w,
        rightB 	= b.x + b.w,
        topA   	= a.y,
        topB   	= b.y,
        bottomA	= a.y + a.h,
        bottomB = b.y + b.h;
    // --
    if ( (bottomA <= topB+offsetY) || (topA >= bottomB-offsetY) || (rightA <= leftB+offsetX) || (leftA >= rightB-offsetX) ) return false;
    return true;
}

uint32_t getpixel(SDL_Surface *surface, int x, int y) {
    int bpp = surface->format->BytesPerPixel;
    if ((x < 0) || (y < 0) || (x >= surface->w) || (y >= surface->h)) return 0;
    uint8_t *pixel = (uint8_t *)surface->pixels + y * surface->pitch + x * bpp;
    switch(bpp) {
        case 1:	return *pixel; 						break;
        case 2: return *(uint16_t *)pixel;        	break;
        case 3: /*if (SDL_BYTEORDER != SDL_BIG_ENDIAN)*/ return pixel[0] << 16 | pixel[1] << 8 | pixel[2]; /*else return pixel[0] | pixel[1] << 8 | pixel[2] << 16;*/ break;
        case 4: return *(uint32_t *)pixel;			break;
        default: return 0;
    }
}

Mix_Music  * loadMusic(const char * filename) {
    Mix_Music * music = Mix_LoadMUS(filename);
    if (music == NULL) SDL_Log("Mix_LoadMUS Error: %s\r\n", Mix_GetError());
    return music;
}

Mix_Chunk  * loadSound(const char * filename) {
    Mix_Chunk * music = Mix_LoadWAV(filename);
    if (music == NULL) SDL_Log("Mix_LoadWAV Error: %s\r\n", Mix_GetError());
    return music;
}

// ------------------------------------------------------------------------------------------------

#define WALL_IMPACT_SOUND	0x00
#define GET_ITEM_SOUND		0x01
#define MONSTER_HURT_SOUND	0x02
#define PLAYER_HURT_SOUND	0x03

class Sounds {
public:
	Sounds(const char * wallImpactSound, const char * getItemSound, const char * monsterHurtSound, const char * playerHurtSound) {
		this->sounds[0] = loadSound(wallImpactSound);
		this->sounds[1] = loadSound(getItemSound);
		this->sounds[2] = loadSound(monsterHurtSound);
		this->sounds[3] = loadSound(playerHurtSound);
	}
	~Sounds() {
		for (int i = 0; i < 4; i++) 
			if (this->sounds[i] != NULL)
				Mix_FreeChunk(this->sounds[i]);
	}
	void playSound(int type) {
		if ((type > -1) && (type < 4))
			if (this->sounds[type] != NULL)
				Mix_PlayChannel(-1, this->sounds[type], 0);	
	}
private:
	Mix_Chunk *sounds[4];
};

// ------------------------------------------------------------------------------------------------

#define PLAYERTILE_W	32
#define PLAYERTILE_H	32

#define MONSTERTILE_W	32
#define MONSTERTILE_H	32

#define TILE_SIZE_X		32
#define TILE_SIZE_Y		32
#define FPF				6

#define UP				0
#define UP_RIGHT		1
#define RIGHT			2
#define DOWN_RIGHT		3
#define DOWN			4
#define DOWN_LEFT		5
#define LEFT			6
#define UP_LEFT			7

#define IMPACT_WALL		19
#define IMPACT_MONSTER	13

class Base {
public:
    Base(int x, int y, int w, int h, int type, bool animation, int numFrames) {
        this->tileRect.w	= w;
        this->tileRect.h	= h;
		this->tileRect.x	= x;
		this->tileRect.y	= y;
        this->type			= type;
        this->animation 	= animation;
        this->numFrames		= numFrames;
        this->fpf			= FPF;
        this->frame			= 0;        
    }
    ~Base() {}
    void renderize(SDL_Renderer* renderer, SDL_Rect camera, SDL_Rect srcRect, SDL_Texture * monsterSheet, int screenWidth, int screenHeight) {
        SDL_Rect dstRect;
        dstRect.x = this->tileRect.x - camera.x;
        dstRect.y = this->tileRect.y - camera.y;
        dstRect.w = this->tileRect.w;
        dstRect.h = this->tileRect.h;
        if ( (dstRect.x > -dstRect.w) && (dstRect.y > -dstRect.h) && (dstRect.x < screenWidth) && (dstRect.y < screenHeight) )
            SDL_RenderCopy(renderer, monsterSheet, &srcRect, &dstRect);
    }
    SDL_Rect getRect(void) { return this->tileRect; }
protected:
    SDL_Rect tileRect;
	int type, numFrames, fpf, frame;
    bool animation;
};

// ------------------------------------------------------------------------------------------------

class Explosion : protected Base {
public:
    Explosion(int x, int y, int w, int h, int type) : Base (x * w, y * h, w, h, type, true, (type==IMPACT_MONSTER?6:4)) { }
    ~Explosion() {}
    bool update(void) {
    	return (this->frame > fpf * this->numFrames ? true : false);
	}
    void render(SDL_Renderer* renderer, SDL_Rect camera, SDL_Texture * monsterSheet, int screenWidth, int screenHeight) {
        SDL_Rect srcRect;
    	srcRect.x = TILE_SIZE_X * (this->type + ((int)(round(++this->frame/this->fpf))%this->numFrames));			
	    srcRect.y = TILE_SIZE_Y * 9;
        srcRect.w = TILE_SIZE_X; // TILE SIZE
        srcRect.h = TILE_SIZE_Y; // TILE SIZE
        this->renderize(renderer, camera, srcRect, monsterSheet, screenWidth, screenHeight);
    }
    int getType(void) const { return this->type; }
private:
	
};

// ------------------------------------------------------------------------------------------------

class Treasure : protected Base {
public:
    Treasure(int x, int y, int w, int h, int type) : Base (x * w, y * h, w, h, type, (type==7?true:false), (type==7?3:0)) { }
    ~Treasure() {}
    bool update(SDL_Rect playerRect) {
		if (checkCollision(playerRect, this->tileRect)) return true;
		return false;
	}
    void render(SDL_Renderer* renderer, SDL_Rect camera, SDL_Texture * monsterSheet, int screenWidth, int screenHeight) {
        SDL_Rect srcRect;
        if (this->animation == true) 
			srcRect.x = TILE_SIZE_X * (this->type + ((int)(round(++this->frame/this->fpf))%this->numFrames));			
		else
			srcRect.x = TILE_SIZE_X * this->type;
        srcRect.y = TILE_SIZE_Y * 9;
        srcRect.w = TILE_SIZE_X; // TILE SIZE
        srcRect.h = TILE_SIZE_Y; // TILE SIZE
        this->renderize(renderer, camera, srcRect, monsterSheet, screenWidth, screenHeight);
    }
    int getType(void) const { return this->type; }
private:
	
};

// ------------------------------------------------------------------------------------------------

class Exit : protected Base {
public:
    Exit(int x, int y, int w, int h) : Base (x * w, y * h, w, h, 0, false, 0) { }
    ~Exit() {}
    bool update(SDL_Rect playerRect) {
		if (checkCollision(playerRect, this->tileRect, this->tileRect.w-1, this->tileRect.h-1))	return true;
		return false;
	}
    void render(SDL_Renderer* renderer, SDL_Rect camera, SDL_Texture * monsterSheet, int screenWidth, int screenHeight) {
        SDL_Rect srcRect;
        srcRect.x = TILE_SIZE_X * 12;
        srcRect.y = TILE_SIZE_Y * 9;
        srcRect.w = TILE_SIZE_X; // TILE SIZE
        srcRect.h = TILE_SIZE_Y; // TILE SIZE
        this->renderize(renderer, camera, srcRect, monsterSheet, screenWidth, screenHeight);
    }
private:
};

// ------------------------------------------------------------------------------------------------

class Door : protected Base {
public:
    Door(int x, int y, int w, int h, bool vh) : Base (x * w, y * h, w, h, vh, false, 0) { }
    ~Door() {}
    void update(void) {}
    void render(SDL_Renderer* renderer, SDL_Rect camera, SDL_Texture * monsterSheet, int screenWidth, int screenHeight) {
        SDL_Rect srcRect;
        srcRect.x = TILE_SIZE_X * (10 + this->type);
        srcRect.y = TILE_SIZE_Y * 9;
        srcRect.w = TILE_SIZE_X; // TILE SIZE
        srcRect.h = TILE_SIZE_Y; // TILE SIZE
        this->renderize(renderer, camera, srcRect, monsterSheet, screenWidth, screenHeight);
    }
private:
};

// ------------------------------------------------------------------------------------------------

class Generator : protected Base {
public:
    Generator(int x, int y, int w, int h, int type) : Base (x * w, y * h, w, h, type, false, 3) { }
    ~Generator() {}
    void update(void) {}
    void render(SDL_Renderer* renderer, SDL_Rect camera, SDL_Texture * monsterSheet, int screenWidth, int screenHeight) {
        SDL_Rect srcRect;
        srcRect.x = TILE_SIZE_X * 32;
        srcRect.y = TILE_SIZE_Y * (4 + this->type);
        srcRect.w = TILE_SIZE_X; // TILE SIZE
        srcRect.h = TILE_SIZE_Y; // TILE SIZE
        this->renderize(renderer, camera, srcRect, monsterSheet, screenWidth, screenHeight);
    }
private:
};

// ------------------------------------------------------------------------------------------------

class ThrownWeapon : public Base {
public:
    ThrownWeapon(int x, int y, int w, int h, int type, int destX, int destY) : Base (x * w, y * h, w, h, type, true, 8) { 
		this->dstX = destX * this->tileRect.w;
		this->dstY = destY * this->tileRect.h;
		this->incX = (float)(destX - x) * (this->tileRect.w/16); 
		if ((destX - x) != 0) 
			this->incY = fabs(incX) * (float)((float)(destY - y) / (float)abs(destX - x)); 
		else
			this->incY = (float)((float)(destY - y) * (this->tileRect.w/16)); 
		
		this->accY = this->tileRect.y;
		this->accX = this->tileRect.x;
	}
    ~ThrownWeapon() {}
    bool update(void) {
    	this->accY += this->incY;
    	this->accX += this->incX;
    	this->tileRect.x = this->accX;
    	this->tileRect.y = this->accY;
		if ((abs(this->tileRect.x - this->dstX) < 5) && (abs(this->tileRect.y - this->dstY) < 5)) return true;
		return false;
	}
    void render(SDL_Renderer* renderer, SDL_Rect camera, SDL_Texture * monsterSheet, int screenWidth, int screenHeight) {
        SDL_Rect srcRect;
		srcRect.x = TILE_SIZE_X * (24 + ((int)(round(++this->frame/this->fpf))%this->numFrames));	
        srcRect.y = TILE_SIZE_Y * this->type; // It depends on Player type
        srcRect.w = TILE_SIZE_X; // TILE SIZE
        srcRect.h = TILE_SIZE_Y; // TILE SIZE
        this->renderize(renderer, camera, srcRect, monsterSheet, screenWidth, screenHeight);
    }
private:
	int dstX, dstY;
	float incY, incX, accX, accY;
};

// ------------------------------------------------------------------------------------------------

#define AREATILES	5

class Monster : public Base {
public:
    Monster(int x, int y, int w, int h, int type) : Base (x * w, y * h, w, h, type, true, 3) { this->usualDirection = x % 2; this->displacement = 0; }
    ~Monster() {}
    bool update(SDL_Rect playerRect, bool isTouchingWall, SDL_Rect camera, int screenWidth, int screenHeight) {
        if ( (this->tileRect.x - camera.x > -this->tileRect.w) && (this->tileRect.y - camera.y > -this->tileRect.h) && (this->tileRect.x - camera.x < screenWidth) && (this->tileRect.y - camera.y < screenHeight) ) {
			if (checkCollision(playerRect, this->tileRect))	return true;
			if (isTouchingWall == true) {
		        switch (this->direction) {
	                case LEFT: 	 	this->tileRect.x += this->tileRect.w/16;	break;
	                case UP: 		this->tileRect.y += this->tileRect.h/16;	break;
	                case RIGHT: 	this->tileRect.x -= this->tileRect.w/16;	break;
	                case DOWN: 		this->tileRect.y -= this->tileRect.h/16;	break;
	                case UP_LEFT: 	this->tileRect.y += this->tileRect.h/16;	this->tileRect.x += this->tileRect.w/16;	break;
	                case UP_RIGHT: 	this->tileRect.y += this->tileRect.h/16;	this->tileRect.x -= this->tileRect.w/16;	break;
	                case DOWN_RIGHT:this->tileRect.y -= this->tileRect.h/16;	this->tileRect.x -= this->tileRect.w/16;	break;
	                case DOWN_LEFT: this->tileRect.y -= this->tileRect.h/16;	this->tileRect.x += this->tileRect.w/16;	break;
	    	    }
			}
			SDL_Rect rect;
	        rect.x = this->tileRect.x - this->tileRect.w * AREATILES;
	        rect.y = this->tileRect.y - this->tileRect.h * AREATILES;
	        rect.w = this->tileRect.w * (AREATILES * 2 + 1);
	        rect.h = this->tileRect.h * (AREATILES * 2 + 1);
			if (checkCollision(rect, playerRect)) { // if player is near
				this->direction = 0;
				if (playerRect.x < this->tileRect.x) { this->tileRect.x -= this->tileRect.w/16; this->direction |= 0x01; }
				if (playerRect.y < this->tileRect.y) { this->tileRect.y -= this->tileRect.h/16; this->direction |= 0x02; }	
				if (playerRect.x > this->tileRect.x) { this->tileRect.x += this->tileRect.w/16; this->direction |= 0x04; }
				if (playerRect.y > this->tileRect.y) { this->tileRect.y += this->tileRect.h/16; this->direction |= 0x08; }
		        switch (this->direction) {
	                case 0x01: this->direction = LEFT; 		break;
	                case 0x02: this->direction = UP; 		break;
	                case 0x04: this->direction = RIGHT;		break;
	                case 0x08: this->direction = DOWN; 		break;
	                case 0x03: this->direction = UP_LEFT;	break;
	                case 0x06: this->direction = UP_RIGHT;	break;
	                case 0x0C: this->direction = DOWN_RIGHT;break;
	                case 0x09: this->direction = DOWN_LEFT;	break;
	    	    }
	    	    this->displacement = 0;
	    	} else {
	    		if (this->usualDirection == 0) {
	    			if (this->direction == LEFT) {
						if (this->displacement > -32) {
							this->tileRect.x -= this->tileRect.w/16;
	    					this->direction = LEFT;
	    					displacement--;
	    				} else {
	    					this->direction = RIGHT;
						}
					} else {
						if (this->displacement < 32) {
							this->tileRect.x += this->tileRect.w/16;
	    					this->direction = RIGHT;
	    					displacement++;
	    				} else {
	    					this->direction = LEFT;
						}				
					}
				} else {
	    			if (this->direction == DOWN) {
						if (this->displacement > -32) {
							this->tileRect.y += this->tileRect.h/16;
	    					this->direction = DOWN;
	    					displacement--;
	    				} else {
	    					this->direction = UP;
						}
					} else {
						if (this->displacement < 32) {
							this->tileRect.y -= this->tileRect.h/16;
	    					this->direction = UP;
	    					displacement++;
	    				} else {
	    					this->direction = DOWN;
						}				
					}
				}
			}
		}
		return false;
	}
    void render(SDL_Renderer* renderer, SDL_Rect camera, SDL_Texture * monsterSheet, int screenWidth, int screenHeight) {
        SDL_Rect srcRect;
        srcRect.x = (this->direction + 8 * ((int)(round(++this->frame/this->fpf))%this->numFrames)) * MONSTERTILE_W;
        srcRect.y = MONSTERTILE_H * this->type + 4 * MONSTERTILE_H;
        srcRect.w = MONSTERTILE_W; // TILE SIZE
        srcRect.h = MONSTERTILE_H; // TILE SIZE
        this->renderize(renderer, camera, srcRect, monsterSheet, screenWidth, screenHeight);
    }
private:
	int direction, usualDirection, displacement;
};

// ------------------------------------------------------------------------------------------------

class Tile {
public:
    Tile (int x, int y, int w, int h, bool wall, int tileType, int shadowType, bool nothing) {
        this->tileRect.x = x;
        this->tileRect.y = y;
        this->tileRect.w = w;
        this->tileRect.h = h;
        this->tileType   = tileType;
        this->shadowType = shadowType;
        this->nothing	 = nothing;
        this->wall		 = wall;
    }
    void render(SDL_Renderer* renderer, SDL_Rect camera, SDL_Texture * tileSheet, int screenWidth, int screenHeight, int level) {
        SDL_Rect dstRect, srcRect;
        dstRect.x = this->tileRect.x - camera.x;
        dstRect.y = this->tileRect.y - camera.y;
        dstRect.w = this->tileRect.w;
        dstRect.h = this->tileRect.h;
        if (this->nothing == false) {
            if (this->wall == true) {
                srcRect.x = this->tileType * TILE_SIZE_X;
                srcRect.y = TILE_SIZE_Y * level;
            } else {
                srcRect.x = TILE_SIZE_X * level;
                srcRect.y = 0;
            }
        } else {
            srcRect.x = 0;
            srcRect.y = 0;
        }
        srcRect.w = TILE_SIZE_X; // TILE SIZE
        srcRect.h = TILE_SIZE_Y; // TILE SIZE
        if ( (dstRect.x > -dstRect.w) && (dstRect.y > -dstRect.h) && (dstRect.x < screenWidth) && (dstRect.y < screenHeight) )
            SDL_RenderCopy(renderer, tileSheet, &srcRect, &dstRect);
        // Draw Shadows
        if ((this-shadowType != 0) && (this->wall == false)) {
            srcRect.x = this->shadowType * TILE_SIZE_X;
            srcRect.y = TILE_SIZE_Y * 7;
            srcRect.w = TILE_SIZE_X; // TILE SIZE
            srcRect.h = TILE_SIZE_Y; // TILE SIZE
            if ( (dstRect.x > -dstRect.w) && (dstRect.y > -dstRect.h) && (dstRect.x < screenWidth) && (dstRect.y < screenHeight) )
                SDL_RenderCopy(renderer, tileSheet, &srcRect, &dstRect);
        }
    }
    bool isWall(void) const { return this->wall; }
    int getType(void) const { return this->tileType; }
    SDL_Rect getRect(void) const { return this->tileRect; }
private:
    SDL_Rect tileRect;
    int tileType, shadowType;
    bool nothing, wall;
};

// ------------------------------------------------------------------------------------------------

#define PIXEL_NOTHING	0x000000 // BLACK
#define PIXEL_DOOR		0xC0C000 // YELLOW
#define PIXEL_WALL		0x404000 // DARK YELLOW
#define PIXEL_GENERATOR 0xF00000 // RED
#define PIXEL_MONSTER	0x400000 // DARK RED
#define PIXEL_START		0x00F000 // GREEN
#define PIXEL_TREASURE	0x008000 // MEDIUM GREEN
#define PIXEL_EXIT		0x004000 // DARK GREEN
#define PIXEL_MASK_TYPE 0xFFFF00

class TileMap {
public:
    TileMap(std::string filename, int sideX, int sideY, SDL_Texture * tileSheet, int screenWidth, int screenHeight) {
        this->tilesLoaded 	= true;
        this->sideX 		= sideX;
        this->sideY 		= sideY;
        this->tileSheet 	= tileSheet;
        this->screenWidth 	= screenWidth;
        this->screenHeight	= screenHeight;
        uint32_t tileType, shadowType, pixel;
        // --
        SDL_Surface * map = IMG_Load(filename.c_str());
        if (map == NULL) {
            this->tilesLoaded = false;
            return;
        }
        this->maxXtiles = map->w;
        this->maxYtiles = map->h;
        this->grid = std::vector< std::vector<int> > (this->maxYtiles, std::vector<int>(this->maxXtiles));
        // TODO ARREGLAR ESTO
        for (int x = 0; x < map->w; x++) {
            for (int y = 0; y < map->h; y++) {
                pixel = getpixel(map, x, y);
                // Player Start Position
                if (isStart(pixel)) { this->startPlayerX = x; this->startPlayerY = y; }
                // Wall or not wall
                if (this->isWall(pixel)) grid[y][x] = 0; else grid[y][x] = 1;
                // TYheris an exit
                if (this->isExit(pixel)) this->exits.push_back(Exit(x, y, this->sideX, this->sideY));
                // There is a door?
                else if (this->isDoor(pixel)) this->doors.push_back(Door(x, y, this->sideX, this->sideY, (this->isWall(getpixel(map, x, (y-1))) || this->isDoor(getpixel(map, x, (y-1))))));
                // There is monster generator
                else if (this->isGenerator(pixel)) this->generators.push_back(Generator(x, y, this->sideX, this->sideY, ((pixel&0x0000F0)>>4)));
                // There is a treasure?
                else if (this->isTreasure(pixel)) this->treasures.push_back(Treasure(x, y, this->sideX, this->sideY, ((pixel&0x0000F0)>>4)));
                // There is a monster?
                else if (this->isMonster(pixel)) this->monsters.push_back(Monster(x, y, this->sideX, this->sideY, ((pixel&0x0000F0)>>4)));
                // Wall type
                tileType = 	 (this->isWall(getpixel(map, x, (y-1)))?1:0) |
                               (this->isWall(getpixel(map, (x+1), y))?2:0) |
                               (this->isWall(getpixel(map, x, (y+1)))?4:0) |
                               (this->isWall(getpixel(map, (x-1), y))?8:0);
                // Shadow type
                shadowType = (this->isWall(getpixel(map, (x-1), y))?1:0) |
                             (this->isWall(getpixel(map, (x-1), (y+1)))?2:0) |
                             (this->isWall(getpixel(map,  x, (y+1)))?4:0);
                this->tiles.push_back(Tile (x * sideX, y * sideY, sideX, sideY, this->isWall(pixel), tileType, shadowType, this->isNothing(pixel)));   // Modificar TILE para que imprima el correcto
            }
        }
        SDL_FreeSurface(map);
    }
    ~TileMap() { tiles.clear();	grid.clear(); monsters.clear(); treasures.clear(); doors.clear(); exits.clear(); generators.clear(); }
    void render(SDL_Renderer* renderer, SDL_Rect camera) {
        for (int i = 0; i < this->maxXtiles * this->maxYtiles; i++)
            this->tiles[i].render(renderer, camera, this->tileSheet, this->screenWidth, this->screenHeight, 2);
    }
    bool touchesWall(SDL_Rect rect) {
        for (int i = 0;  i < this->maxXtiles * this->maxYtiles; i++)
            if (this->tiles[i].isWall())
                if (checkCollision(rect, this->tiles[i].getRect()))
                    return true;
        return false;
    }
    bool touchesWall(int x, int y, int w, int h) {
        SDL_Rect rect = { .x = x, .y = y, .w = w, .h = h };
        return this->touchesWall(rect);
    }
	std::vector< std::vector<int> > getGrid(void) const { return this->grid; 	}
    std::vector<Monster> * getMonsters(void) 		{ return &this->monsters; 	}
    std::vector<Generator> * getGenerators(void)  	{ return &this->generators; }
    std::vector<Treasure> * getTreasures(void)  	{ return &this->treasures; 	}
    std::vector<Door> * getDoors(void) 				{ return &this->doors;		}
    std::vector<Exit> * getExits(void) 				{ return &this->exits; 		}
    int getLevelWidth(void) const 	{ return (this->maxXtiles * this->sideX); 	}
    int getLevelHeight(void) const 	{ return (this->maxYtiles * this->sideY); 	}
    int getSideX(void) const 		{ return this->sideX; 						}
    int getSideY(void) const 		{ return this->sideY; 						}
    int getNumCols(void) const 		{ return this->maxXtiles; 					}
    int getNumRows(void) const 		{ return this->maxYtiles; 					}
    int getPlayerStartX(void) const	{ return this->startPlayerX * this->sideX; 	}
    int getPlayerStartY(void) const	{ return this->startPlayerY * this->sideY; 	}
private:
    bool isWall(uint32_t pixel) 	{ return ((pixel&PIXEL_MASK_TYPE)==PIXEL_WALL)?true:false; 		}
    bool isNothing(uint32_t pixel) 	{ return ((pixel&PIXEL_MASK_TYPE)==PIXEL_NOTHING)?true:false; 	}
    bool isStart(uint32_t pixel) 	{ return ((pixel&PIXEL_MASK_TYPE)==PIXEL_START)?true:false; 	}
    bool isDoor(uint32_t pixel) 	{ return ((pixel&PIXEL_MASK_TYPE)==PIXEL_DOOR)?true:false; 		}
    bool isExit(uint32_t pixel) 	{ return ((pixel&PIXEL_MASK_TYPE)==PIXEL_EXIT)?true:false; 		}
    bool isGenerator(uint32_t pixel){ return ((pixel&PIXEL_MASK_TYPE)==PIXEL_GENERATOR)?true:false; }
    bool isMonster(uint32_t pixel)	{ return ((pixel&PIXEL_MASK_TYPE)==PIXEL_MONSTER)?true:false; 	}
    bool isTreasure(uint32_t pixel)	{ return ((pixel&PIXEL_MASK_TYPE)==PIXEL_TREASURE)?true:false; 	}
    // --
    std::vector<Monster> monsters;
    std::vector<Generator> generators;
    std::vector<Treasure> treasures;
    std::vector<Door> doors;
    std::vector<Exit> exits;
    int maxXtiles, maxYtiles;
    std::vector<Tile> tiles;
    int sideX, sideY;
    int screenWidth, screenHeight;
    bool tilesLoaded;
    SDL_Texture * tileSheet;
    std::vector< std::vector<int> > grid;
    int startPlayerX, startPlayerY;
};

// ------------------------------------------------------------------------------------------------

class Player {
public:
    Player(int x, int y, int w, int h, SDL_Texture * playerSheet) {
        this->playerX 			= x;
        this->playerY 			= y;
        this->playerRect.x	  	= x;
        this->playerRect.y	  	= y;
        this->playerRect.w	  	= w;
        this->playerRect.h	  	= h;
        this->playerSheet   	= playerSheet;
        this->direction			= RIGHT;
        this->playerSpriteRect.x= this->direction * PLAYERTILE_W;
        this->playerSpriteRect.y= PLAYERTILE_W;
        this->playerSpriteRect.w= PLAYERTILE_W;
        this->playerSpriteRect.h= PLAYERTILE_H;
        this->animation 		= false;
        this->numFrames			= 3;
        this->fpf				= FPF;
        this->frame				= 0;
        this->animationCount 	= 8;
    }
    ~Player() { this->thrownWeapeons.clear(); }
    void setCamera(SDL_Rect & camera, int screenWidth, int screenHeight, int levelWidth, int levelHeight) {
        // Place player in center of the camera
        camera.x = (this->playerRect.x + this->playerRect.w / 2 ) - screenWidth / 2;
        camera.y = (this->playerRect.y + this->playerRect.h / 2 ) - screenHeight / 2;
        // Check limits
        if (camera.x < 0) camera.x = 0;
        if (camera.y < 0) camera.y = 0;
        if (camera.x > levelWidth - camera.w) camera.x = levelWidth - camera.w;
        if (camera.y > levelHeight - camera.h) camera.y = levelHeight - camera.h;
    }
    void render(SDL_Renderer* renderer, SDL_Rect camera) {
        SDL_Rect dstRect;
        dstRect.x = this->playerRect.x - camera.x;
        dstRect.y = this->playerRect.y - camera.y;
        dstRect.w = this->playerRect.w;
        dstRect.h = this->playerRect.h;
        if (this->animation == true)
            this->playerSpriteRect.x = (this->direction + 8 * ((int)(round(++this->frame/this->fpf))%this->numFrames)) * PLAYERTILE_W;
        else
            this->frame = 0;
        SDL_RenderCopy(renderer, this->playerSheet, &this->playerSpriteRect, &dstRect);
    }
    bool update(SDL_Event event, TileMap mapTiles, SDL_Rect camera, int screenWidth, int screenHeight) {
        int direction = 0x00;
		this->animation = true;
        bool enemyTouched = false;
        if ( (this->playerX != this->playerRect.x) || (this->playerY != this->playerRect.y) ) {
            if (this->playerX < this->playerRect.x) { this->playerRect.x -= mapTiles.getSideX()/8; direction |= 0x01; }
            if (this->playerY < this->playerRect.y) { this->playerRect.y -= mapTiles.getSideY()/8; direction |= 0x02; }
            if (this->playerX > this->playerRect.x) { this->playerRect.x += mapTiles.getSideX()/8; direction |= 0x04; }
            if (this->playerY > this->playerRect.y) { this->playerRect.y += mapTiles.getSideY()/8; direction |= 0x08; }
            switch (direction) {
                case 0x01: this->direction = LEFT; 		break;
                case 0x02: this->direction = UP; 		break;
                case 0x04: this->direction = RIGHT;		break;
                case 0x08: this->direction = DOWN; 		break;
                case 0x03: this->direction = UP_LEFT;	break;
                case 0x06: this->direction = UP_RIGHT;	break;
                case 0x0C: this->direction = DOWN_RIGHT;break;
                case 0x09: this->direction = DOWN_LEFT;	break;
            }
        } else {
        	int srcX  = ((this->playerX) / mapTiles.getSideX()),
				srcY  = ((this->playerY) / mapTiles.getSideY());
            while (SDL_PollEvent(&event) != 0) {
                if (event.type == SDL_QUIT) return true;
                if ((event.type == SDL_MOUSEBUTTONUP) || (event.type == SDL_FINGERUP) ) {
#ifdef _WIN32
                    if (event.button.button == SDL_BUTTON_LEFT) {
                        int x = event.button.x,
							y = event.button.y;
#else
                        int x = (event.tfinger.x * screenWidth),
                            y = (event.tfinger.y * screenHeight);
                        if ((x > 0) && (x < screenWidth) && (y > 0) && (y < screenHeight)) { // If X and Y coordinates are OK
#endif
	                        int 	dstX  = ((x + camera.x) / mapTiles.getSideX()),
                                	dstY  = ((y + camera.y) / mapTiles.getSideY());
                            Pair 	src   = std::make_pair(srcY, srcX),
                                	dest  = std::make_pair(dstY, dstX);

							// Check if monster is selected in order to attack 
							if (mapTiles.getMonsters()->empty() == false) 	
								for (int i = mapTiles.getMonsters()->size() - 1; i >= 0; i--) 
									if ( ((x + camera.x) > mapTiles.getMonsters()->at(i).getRect().x - mapTiles.getMonsters()->at(i).getRect().w/2) &&
							    		 ((x + camera.x) < mapTiles.getMonsters()->at(i).getRect().x + mapTiles.getMonsters()->at(i).getRect().w*3/2) &&
										 ((y + camera.y) > mapTiles.getMonsters()->at(i).getRect().y - mapTiles.getMonsters()->at(i).getRect().h/2) &&
							    		 ((y + camera.y) < mapTiles.getMonsters()->at(i).getRect().y + mapTiles.getMonsters()->at(i).getRect().h*3/2) ) {
										enemyTouched = true;
										this->animationCount = 0;
										//std::cout << "Lanzada arma" << std::endl;
										this->thrownWeapeons.push_back(ThrownWeapon(srcX, srcY, this->playerRect.w, this->playerRect.h, 1, dstX, dstY));
										direction = 0x00;
										if (dstX < srcX) direction |= 0x01;
										if (dstX > srcX) direction |= 0x04;
										if (dstY < srcY) direction |= 0x02;
										if (dstY > srcY) direction |= 0x08;
										switch (direction) {
										    case 0x01: this->direction = LEFT; 		break;
										    case 0x02: this->direction = UP; 		break;
										    case 0x04: this->direction = RIGHT;		break;
										    case 0x08: this->direction = DOWN; 		break;
										    case 0x03: this->direction = UP_LEFT;	break;
										    case 0x06: this->direction = UP_RIGHT;	break;
										    case 0x0C: this->direction = DOWN_RIGHT;break;
										    case 0x09: this->direction = DOWN_LEFT;	break;
										}
										break;
									}
							if (enemyTouched == false) {
								this->clearPath();
								if (aStarSearch(mapTiles.getGrid(), mapTiles.getNumRows(), mapTiles.getNumCols(), src, dest, this->Path) == false)
   	                		    	SDL_Log("Failed to find the Destination Cell\r\n");
							}   
					}
                }
            }
            if (!this->Path.empty()) { 
                std::pair<int,int> p = Path.top();
                Path.pop();
                // SDL_Log("-> (%d, %d)\r\n", p.first, p.second);
                this->playerY = p.first * mapTiles.getSideY();
                this->playerX = p.second * mapTiles.getSideX();
            } else {
				if (enemyTouched == false)
					if (this->animationCount == 8)
            			this->animation = false;
            		else
            			this->animationCount++;
			}
        }
        return false;
    }
    void addTreasure(int type) {
    	// TODO
	}
	SDL_Rect getPlayerRect(void) const { return this->playerRect; }   
	std::vector<ThrownWeapon> * getThrownWeapons(void) { return &this->thrownWeapeons; }
private:
    void clearPath(void) { while (!this->Path.empty()) this->Path.pop(); }
    // --
    SDL_Rect playerRect, playerSpriteRect;
    SDL_Texture * playerSheet;
    std::stack<Pair> Path;
    std::vector<ThrownWeapon> thrownWeapeons;
    int playerX, playerY;
    uint8_t direction, numFrames, fpf;
    int frame, animationCount;
    bool animation;
    
};

///////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
    int 	SCREEN_WIDTH    = 320,//1920,
            SCREEN_HEIGHT   = 200;//1080;
    SDL_Event event;
    SDL_DisplayMode displayMode;
	// Init SDL library
    if (SDL_Init(SDL_INIT_EVERYTHING) != 0) {
        SDL_Log("SDL_Init Error: %s\r\n", SDL_GetError());
        return 1;
    }
	// Init IMG library
    if (IMG_Init(IMG_INIT_PNG) != IMG_INIT_PNG) {
        SDL_Log("IMG_Init Error: %s\r\n", IMG_GetError());
        return 1;
    }
    // Init MIX library
#ifndef _WIN32
	if (Mix_Init(MIX_INIT_OGG) != MIX_INIT_OGG) {
       	SDL_Log("Mix_Init Error: %s\r\n", Mix_GetError());
        return 1;
	}
#endif	
	// Initialize SDL_mixer
    if (Mix_OpenAudio(22050, MIX_DEFAULT_FORMAT, 2, 4096 ) < 0) {
		SDL_Log("MIX could not initialize! MIX Error: %s\r\n", Mix_GetError());
        return 1;    
	}
	// Create a new Window
    SDL_Window *win = SDL_CreateWindow("Prueba", 100, 100, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);
    if (win == NULL) {
        SDL_Log("SDL_CreateWindow Error: %s\r\n", SDL_GetError());
        return 1;
    }
	// Create a render
    SDL_Renderer *ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (ren == NULL) {
        SDL_Log("SDL_CreateRenderer Error: %s\r\n", SDL_GetError());
        return 1;
    }
	// Configure window resolution
#ifndef _WIN32
    if (SDL_GetCurrentDisplayMode(0, &displayMode) == 0) {
        SCREEN_WIDTH  = displayMode.w;
        SCREEN_HEIGHT = displayMode.h;
    } else {
        SDL_Log("SDL_GetCurrentDisplayMode Error\r\n");
        return -1;
    }
#endif
	// Texture copy of screen content
	SDL_Texture *texTarget = SDL_CreateTexture(ren, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_TARGET, SCREEN_WIDTH, SCREEN_HEIGHT);
	SDL_Rect rect;
	rect.x = 0;
	rect.y = 0;
	rect.w = SCREEN_WIDTH;
	rect.h = SCREEN_HEIGHT;
	// Load BACKGROUND sprite sheet
    SDL_Texture * spriteSheet = loadSpriteSheet(ren, "backgrounds.png");
    if (spriteSheet == NULL) {
        SDL_Log("SpriteSheet not loaded\r\n");
        return 1;
    }
	// Load ITEMS sprite sheet
    SDL_Texture * playerSheet = loadSpriteSheet(ren, "entities.png");
    if (playerSheet == NULL) {
        SDL_Log("SpriteSheet not loaded\r\n");
        return 1;
	}
	// Music object
	Mix_Music * music = loadMusic("doungeonMusic.ogg");
    if (music != NULL) Mix_PlayMusic(music, -1);
	// Random init
	srand(time(NULL));	
    // Load Levels
	std::vector <std::string> levels;
	levels.push_back("testlevel.png");
    levels.push_back("trainer1.png");
    levels.push_back("trainer2.png");
    levels.push_back("trainer3.png");
	levels.push_back("trainer4.png");
    levels.push_back("trainer5.png");
    levels.push_back("trainer6.png");
    levels.push_back("trainer7.png");
	levels.push_back("level1.png");
    levels.push_back("level2.png");
    levels.push_back("level3.png");
    levels.push_back("level4.png");    
	levels.push_back("level5.png");
    levels.push_back("level6.png");
    levels.push_back("level7.png");
    levels.push_back("level8.png");
	levels.push_back("level9.png");
	levels.push_back("level10.png");
	// Calculate side dimension in pixels multiple of 8
	int side = findNearMultipleof(SCREEN_HEIGHT/8, 8), level = 0;
	do {
		// Create an object of TileMap class
	    TileMap tileMap(levels.at(level).c_str(), side, side, spriteSheet, SCREEN_WIDTH, SCREEN_HEIGHT);
		// Create an object of Player class
	    Player player(tileMap.getPlayerStartX(), tileMap.getPlayerStartY(), side, side, playerSheet);
		// Create an object of explosions
		std::vector<Explosion> explosions;
		// Create an object of Souns
		Sounds sounds("wallImpactSound.wav", "getItemSound.wav", "monsterHurtSound.wav", "playerHurtSound.wav");
		// Create an object of Camera	
	    SDL_Rect camera = { 0, 0, SCREEN_WIDTH, SCREEN_HEIGHT };
		// Time objects to control FPS
	    uint32_t currentTime, lastUpdateTime = SDL_GetTicks();
		// Shake effect object
		Shake shakeX(3000, 30), shakeY(3000, 30);
		// fade effect object
		Fade fade(BLACK_COLOR, false, 3000);
		fade.start();
		// Exit
		bool exit = false;
	    // Game Loop
	    while ((!exit) || (fade.isFinished() == false)) {
			// Set render target on texture
			SDL_SetRenderTarget(ren, texTarget);
			// Update game objects
			if (!exit) {
				exit = player.update(event, tileMap, camera, SCREEN_WIDTH, SCREEN_HEIGHT);
		        player.setCamera(camera, SCREEN_WIDTH, SCREEN_HEIGHT, tileMap.getLevelWidth(), tileMap.getLevelHeight());
		        // TODO
		        if (player.getThrownWeapons()->empty() == false) 
		        	for (int i = player.getThrownWeapons()->size() - 1; i >= 0; i--) {
		        		if (player.getThrownWeapons()->at(i).update() == true) {
			       			player.getThrownWeapons()->erase(player.getThrownWeapons()->begin() + i); 
						} else {
			        		if (tileMap.touchesWall(player.getThrownWeapons()->at(i).getRect())	== true) {
			        			explosions.push_back(Explosion( player.getThrownWeapons()->at(i).getRect().x/player.getThrownWeapons()->at(i).getRect().w,
			        											player.getThrownWeapons()->at(i).getRect().y/player.getThrownWeapons()->at(i).getRect().h,
			        											player.getThrownWeapons()->at(i).getRect().w,
			        											player.getThrownWeapons()->at(i).getRect().h,
																IMPACT_WALL));
								player.getThrownWeapons()->erase(player.getThrownWeapons()->begin() + i); 
								sounds.playSound(WALL_IMPACT_SOUND);		        			
							} else {
		        		    	if (tileMap.getMonsters()->empty() == false) {
									for (int j = tileMap.getMonsters()->size() - 1; j >= 0; j--)
										if (checkCollision(player.getThrownWeapons()->at(i).getRect(), tileMap.getMonsters()->at(j).getRect(), side/2, side/2) == true) {
						        			explosions.push_back(Explosion( player.getThrownWeapons()->at(i).getRect().x / player.getThrownWeapons()->at(i).getRect().w,
						        											player.getThrownWeapons()->at(i).getRect().y / player.getThrownWeapons()->at(i).getRect().h,
						        											player.getThrownWeapons()->at(i).getRect().w,
						        											player.getThrownWeapons()->at(i).getRect().h,
																			IMPACT_MONSTER));
											tileMap.getMonsters()->erase(tileMap.getMonsters()->begin() + j); 
											player.getThrownWeapons()->erase(player.getThrownWeapons()->begin() + i); 
											sounds.playSound(MONSTER_HURT_SOUND);		        			
											break;
										}						
								}
							}
						}
					}
		        if (tileMap.getMonsters()->empty() == false) 	
					for (int i = tileMap.getMonsters()->size() - 1; i >= 0; i--)
						if (tileMap.getMonsters()->at(i).update(player.getPlayerRect(), tileMap.touchesWall(tileMap.getMonsters()->at(i).getRect()), camera, SCREEN_WIDTH, SCREEN_HEIGHT) == true) {
							// TODO:
							sounds.playSound(PLAYER_HURT_SOUND);
							// std::cout << "Player is injured by monster!!!" << std::endl;
						}
		        if (tileMap.getGenerators()->empty() == false) 	
					for (int i = tileMap.getGenerators()->size() - 1; i >= 0; i--) 
						tileMap.getGenerators()->at(i).update();
				if (tileMap.getTreasures()->empty() == false) 	
					for (int i = tileMap.getTreasures()->size() - 1; i >= 0; i--) 
						if (tileMap.getTreasures()->at(i).update(player.getPlayerRect()) == true) {
							//std::cout << tileMap.getTreasures()->at(i).getType() << " type treasure got it!" << std::endl;	
							sounds.playSound(GET_ITEM_SOUND);
							player.addTreasure(tileMap.getTreasures()->at(i).getType());
							tileMap.getTreasures()->erase(tileMap.getTreasures()->begin() + i); 
						}
		        if (tileMap.getDoors()->empty() == false) 		
					for (int i = tileMap.getDoors()->size() - 1; i >= 0; i--) 
						tileMap.getDoors()->at(i).update();
		        if (tileMap.getExits()->empty() == false) 		
					for (int i = tileMap.getExits()->size() - 1; i >= 0; i--) 
						if (tileMap.getExits()->at(i).update(player.getPlayerRect()) == true) {
							// TODO: PLay sound
							// Image effect
							// Exit
							shakeX.start();
							shakeY.start();
							fade.set(BLACK_COLOR, true, 3000);
							fade.start();
							exit = true;
							std::cout << "Exiting..." << std::endl;
						}
				if (explosions.empty() == false) 		
					for (int i = explosions.size() - 1; i >= 0; i--) 
						if (explosions.at(i).update() == true) 
							explosions.erase(explosions.begin() + i); 
		        // --
		        SDL_SetRenderDrawColor(ren, 0xFF, 0xFF, 0xFF, 0xFF);
		        SDL_RenderClear(ren);
		        tileMap.render(ren, camera);
				for (std::vector<Monster>::size_type i = 0; 	i != tileMap.getMonsters()->size();		i++) tileMap.getMonsters()->at(i).render(ren, camera, playerSheet, SCREEN_WIDTH, SCREEN_HEIGHT);
		        for (std::vector<Generator>::size_type i = 0; 	i != tileMap.getGenerators()->size();	i++) tileMap.getGenerators()->at(i).render(ren, camera, playerSheet, SCREEN_WIDTH, SCREEN_HEIGHT);
		        for (std::vector<Treasure>::size_type i = 0; 	i != tileMap.getTreasures()->size(); 	i++) tileMap.getTreasures()->at(i).render(ren, camera, playerSheet, SCREEN_WIDTH, SCREEN_HEIGHT);
		        for (std::vector<Door>::size_type i = 0; 		i != tileMap.getDoors()->size(); 		i++) tileMap.getDoors()->at(i).render(ren, camera, playerSheet, SCREEN_WIDTH, SCREEN_HEIGHT);
		        for (std::vector<Exit>::size_type i = 0; 		i != tileMap.getExits()->size(); 		i++) tileMap.getExits()->at(i).render(ren, camera, playerSheet, SCREEN_WIDTH, SCREEN_HEIGHT);
				for (std::vector<ThrownWeapon>::size_type i = 0;i != player.getThrownWeapons()->size(); i++) player.getThrownWeapons()->at(i).render(ren, camera, playerSheet, SCREEN_WIDTH, SCREEN_HEIGHT);
				for (std::vector<Explosion>::size_type i = 0; 	i != explosions.size(); 				i++) explosions.at(i).render(ren, camera, playerSheet, SCREEN_WIDTH, SCREEN_HEIGHT);				
		        player.render(ren, camera);
			}
			// Shake effect update
			if (shakeX.getIsShaking() == true) rect.x = shakeX.update()*side/2;
			if (shakeY.getIsShaking() == true) rect.y = shakeY.update()*side/2;
			// Fade effect update
	  		fade.update(ren, SCREEN_WIDTH, SCREEN_HEIGHT);		
			// Set screen target on render
			SDL_SetRenderTarget(ren, NULL);
			// Copy target texture to render
			SDL_RenderCopy(ren, texTarget, NULL, &rect);  
			// Display on screen
			SDL_RenderPresent(ren);
			// Game Timers
	        currentTime = SDL_GetTicks();
	        if ( (currentTime - lastUpdateTime) < 20) SDL_Delay(20 - currentTime + lastUpdateTime);
	        lastUpdateTime = SDL_GetTicks();
		}
		level++;
    } while (level < levels.size());
	if (music != NULL) 	Mix_FreeMusic(music);

    SDL_DestroyTexture(playerSheet);
    SDL_DestroyTexture(spriteSheet);
    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
	Mix_CloseAudio();	
    IMG_Quit();
    SDL_Quit();

    return(0);
}

