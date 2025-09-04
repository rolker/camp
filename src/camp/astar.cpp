/************************************************************/
/*    NAME: Sam Reed                                        */
/*    ORGN: UNH CCOM                                        */
/*    FILE: a_star.cpp                                      */
/*    DATE: 3/21/17                                         */
/************************************************************/

#include "astar.h"

namespace astar
{

AStar::AStar(int connectingDistance) 
{
  NeighborsMask(connectingDistance); // Set dx, dy, and num_directions
}

/*
Number of Neighboors one wants to investigate from each cell. A larger
   number of nodes means that the path can be alligned in more directions.

   Connecting_Distance=1-> Path can  be alligned along 8 different direction.
   Connecting_Distance=2-> Path can be alligned along 16 different direction.
   Connecting_Distance=3-> Path can be alligned along 32 different direction.
   Connecting_Distance=4-> Path can be alligned along 56 different direction.
   ETC......

    This function sets (dx,dy) which are vectors containing the coordinates
    of each valid neighbor, given the connecting distance. They are stored in

*/
void AStar::NeighborsMask(int connectingDistance)

{
    int twice = 2*connectingDistance;
    int mid = connectingDistance;
    int r_size = 2*connectingDistance+1;

    m_candidates.clear();

    // Mask of the desired neighbors
    std::vector< std::vector<int> > new_neighbors (r_size, std::vector<int>(r_size,1));

    // Remove the positions that are the same directions
    for (int i=0; i<connectingDistance-1; i++)
    {
        new_neighbors[i][i]=0;
        new_neighbors[twice-i][i]=0;
        new_neighbors[i][twice-i]=0;
        new_neighbors[twice-i][twice-i]=0;
        new_neighbors[mid][i]=0;
        new_neighbors[mid][twice-i]=0;
        new_neighbors[i][mid]=0;
        new_neighbors[twice-i][mid]=0;
    }
    new_neighbors[mid][mid]=0;
        
    // Find the locations of the mask for the new neighbors
    // relative to the current node.
    for (int i=0; i<r_size; i++)
    {
        for (int j=0; j<r_size; j++)
        {
            if (new_neighbors[i][j] == 1)
            {
                m_candidates.push_back(Position(i-connectingDistance, j-connectingDistance));
            }
	}
    }

    std::cout << "NDir:" << m_candidates.size() << std::endl;
}

std::vector<Position> AStar::reconstructPath(std::map<Position,Node> const &nodeMap, Position const &lastPosition)
{
    std::vector<Position> ret;
    Position currentPosition = lastPosition;
    while(currentPosition.x != -1)
    {
        ret.push_back(currentPosition);
        auto mapNode = nodeMap.find(currentPosition);
        if(mapNode == nodeMap.end())
            break;
        currentPosition = mapNode->second.getParentPosition();
    }
    
    std::vector<Position> fwd_ret;
    for(auto p = ret.rbegin(); p != ret.rend(); p++)
        fwd_ret.push_back(*p);
    return fwd_ret;
}


// Check to see if the extended path runs through any obstacles. It also calculates the cost to
// travel to the cell.
double  AStar::extendedPathAverageDepth(Context const &c, Position const& position, Position const & newPosition)
{
    Position deltaPosition = newPosition - position;
    int dX = abs(deltaPosition.x);
    int dY = abs(deltaPosition.y);

    // FIX: This part of Sam's code discards any diagonal cell in the immediate 8
    // neighbors if the adjacent non-diagonal cells are an obstacle. Is this too
    // conservative or does it make sense?
    if (std::max(dX,dY)==1)
    // If the node is a Moore Neighboor, check the cell on either side of desired cell
    {
        // If either cell next to the desired cell is an obstacle, the path is not valid
        if ((c.map->getDepth(position.x,newPosition.y) < c.minDepth) || (c.map->getDepth(newPosition.x,position.y) < c.minDepth))
            return 0.0; // Path is invalid)
        return c.map->getDepth(newPosition.x,newPosition.y);
    }
    else
    // Otherwise check all cells in the path between the two cells
    {
        // calculate the slope and y-intersect of the line between the two points
        double m = (1.0*(deltaPosition.y))/(1.0*(deltaPosition.x));
        double b = position.y - position.x*m;

        // num_points is points per grid cell to investigate to ensure no obstacle is hit.
        int num_points = 5;
        double total_points = 0.0;
        double cummulative_cost = 0.0;
        
        // Segment the grid in the larger dimension
        if (dX < dY)
        {
            total_points = num_points*dY;

            // Cycle through all segmented grid nodes to check for path validity
            //  determine the total water depth traveled through
            for (int j=1; j<=total_points; j++)
            {
                double intermidate = (1.0*dY)*(j*1.0)/total_points;
                if (deltaPosition.y < 0)
                    intermidate = -intermidate;

                double y = position.y+intermidate;
                double x = (y-b)/m;

                // If either the grid node or the next closest node on the line in the x direction
                //  is an obstacle, the path is not valid
                int floor_x = int(floor(x));
                int ceil_x = int(ceil(x));

                // If either cell is an obstacle, the path is not valid
                if ( c.map->getDepth(floor_x,y) < c.minDepth || c.map->getDepth(ceil_x,y) < c.minDepth)
                {
                    return 0.0; // Path is invalid
                }

                cummulative_cost += c.map->getDepth(round(x),round(y));
            }
        }
        else
        {
            total_points = num_points*dX;

            // Cycle through all segmented grid nodes to check for path validity
            //  determine the total water depth traveled through
            for (int j=1; j<total_points; j++)
            {
                double intermidate = (1.0*dX)*(j*1.0)/total_points;
                if (deltaPosition.x < 0)
                    intermidate = -intermidate;
                
                double x = position.x+intermidate;
                double y= m*x+b;

                // If either the grid node or the next closest node on the line in the x direction
                //  is an obstacle, the path is not valid
                int floor_y = int(floor(y));
                int ceil_y = int(ceil(y));
                if ( c.map->getDepth(x,floor_y) < c.minDepth || c.map->getDepth(x,ceil_y) < c.minDepth)
                {
                    return 0.0; // Path is invalid
                }

                cummulative_cost += c.map->getDepth(round(x),round(y));
            }
        }
        // The depth cost for traversing to the proposed cell, is the mean depth
        // of the samples along the line from the parent node to this child one.
        double avg_depth = cummulative_cost/total_points; // average depth in grid cells
        if(avg_depth < c.minDepth)
            return 0.0;
        return avg_depth;
    }
    return 0.0;
}


// This function runs A*. It outputs the generated path
std::vector<Position> AStar::search(Context const &c)
{
    std::map<Position,Node> nodeMap;
    std::priority_queue<Node, std::vector<Node>, std::greater<Node> > frontier;
    
    // Start node
    Node n0(c, c.start, c.map->getDepth(c.start.x, c.start.y), Node());
    frontier.push(n0);
    
    while (!frontier.empty())
    {
        // A* explores from the highest priority node in the frontier
        n0 = frontier.top();
        //n0.updatePriority(xFinish,yFinish);
        frontier.pop(); // Remove the node from the frontier
        Position position = n0.getPosition();
        
        Position positionReStart = position - c.start;
        Position positionReFinish = position - c.finish;
        
        //std::cerr << "node position: " << position.x << ", " << position.y << "(re start: " << positionReStart.x << ", " << positionReStart.y;
        //std::cerr << " dist: " << positionReStart.distanceFromOrigin() << "), (re finish: " << positionReFinish.x << ", " << positionReFinish.y;
        //std::cerr << " dist: " << positionReFinish.distanceFromOrigin() << ")" << std::endl;

        // If it is a new node, explore the node's neighbors
        if (nodeMap.find(position) == nodeMap.end())
        {
            nodeMap[position] = n0;
            
            // Quit searching when you reach the goal state
            if(position == c.finish)
                return reconstructPath(nodeMap,position);
            
            for (const auto candidate: m_candidates)
            {
                Position newPosition = position + candidate;
                // Place the node in the frontier if the neighbor is within the
                //  map dimensions, not an obstacle, and not closed.
                if(newPosition.isWithinBounds(*c.map) && c.map->getDepth(newPosition.x, newPosition.y) > c.minDepth && nodeMap.find(newPosition) == nodeMap.end())
                {
                    // Check to see if the extended path goes through obstacles
                    // Also calculate the average depth from the parent node to this
                    // new candidate node, return this as avg_depth.
                    double averageDepth = extendedPathAverageDepth(c, position, newPosition);
                    if (averageDepth > 0.0)
                        frontier.push(Node(c, newPosition, averageDepth, n0));
                }
            }
        }
    }
    std::cerr << "No path found." << std::endl;
    return std::vector<Position>();
}

// How we are sorting the frontier priority queue 
bool operator<(const Node& lhs, const Node& rhs)
{
  return lhs.F() < rhs.F();
}

// How we are sorting the frontier priority queue 
bool operator>(const Node& lhs, const Node& rhs)
{
  return lhs.F() > rhs.F();
}


bool operator<(const Position &lhs, const Position &rhs)
{
    if (lhs.y < rhs.y)
        return true;
    if (lhs.y == rhs.y)
        return lhs.x < rhs.x;
    return false;
}


} // namespace astar

