#ifndef ASTAR_H_
#define ASTAR_H_
/*
 * astar.h
 *
 *  Created on: Mar 21, 2017
 *      Author: Sam Reed
 */

#include <vector>
#include <string>
#include <ctime>
#include <cmath> // For sqrt and pow
#include <algorithm> // for max_element and sort
#include <queue> // for priority_queue
#include <iostream>
#include "backgroundraster.h"

namespace astar
{

struct Position
{
    Position(int x=-1, int y=-1):x(x),y(y){}
    int x;
    int y;
    
    Position operator-(Position const &other) const
    {
        return Position(x-other.x, y-other.y);
    }

    Position operator+(Position const &other) const
    {
        return Position(x+other.x, y+other.y);
    }

    
    bool operator==(Position const &other) const
    {
        return x == other.x && y == other.y;
    }
    
    bool isWithinBounds(BackgroundRaster const&map) const
    {
        return x >= 0 && y >= 0 && x < map.width() && y < map.height();
    }
    
    double distanceFromOrigin() const
    {
        return sqrt(x*x+y*y);   
    }
    
    double distanceFrom(Position const &other) const
    {
        return ((*this)-other).distanceFromOrigin();
    }
};

bool operator<(const Position &lhs, const Position &rhs);

struct Context
{
    Context():depthWeightValue(0.11)
    {}
    
    BackgroundRaster *map;
    Position start, finish;
    float depthWeightValue;
    double shipDraft;
    double maxDepth;
    double minDepth;
};

/* --------------------------------------------------------------------------
This class is used in A* to store the information on the nodes of the graph.
The cost for each node can be determined with 3 different methods:
	-minimizing the distance traveled
	-combination of the minimum distance traveled and maximizing the
		depth under the ship
	-combination of the minimum distance traveled and maximizing the 
		time to shore
--------------------------------------------------------------------------- */
class Node
{
public:
    Node():m_G(0),m_H(0),m_depth(0)
    {
    }
            
    Node(Context const &c, Position position, double depth, Node const &parent) :m_position(position), m_parentPosition(parent.m_position), m_depth(depth)
    {
        // Calculate the total cost, g, due to a move to this node, as the sum of the
        // cost to get to the parent node, plus the distance to
        // the node and the cost assocated with the average depth to the node.
        m_G = parent.G() + position.distanceFrom(m_parentPosition) + (1 + depthCostfraction(c));
        
        // Estimate the remaining cost to go to the destination (heuristic - straight line distance)
        m_H = position.distanceFrom(c.finish);
    }
        
    ~Node() {}
    
    bool updateParent(Context const &c, Node const & potentialNewParent)
    {
        double potentialNewG = potentialNewParent.G() + m_position.distanceFrom(potentialNewParent.getPosition()) + (1 + depthCostfraction(c));
        if(potentialNewG < m_G)
        {
            m_parentPosition = potentialNewParent.getPosition();
            m_G = potentialNewG;
            return true;
        }
        return false;
    }

    Position const &getPosition() const { return m_position;}
    Position const &getParentPosition() const { return m_parentPosition;}
    
    double G() const { return m_G;}
    double H() const { return m_H;}
    double F() const { return m_G+m_H;}

    double getDepth() const {return m_depth; }

    // Calculates the cost of traveling through the cells depth
    double depthCostfraction(Context const &c) 
    {
        if (m_depth < c.maxDepth)
            return c.depthWeightValue*(c.maxDepth - m_depth);
        return 0.0;
    }

private:
    Position m_position;
    Position m_parentPosition;
    
    // F = G + H where F is the total cost of the node, G is the distance between the current node
    // and the start node and H is the heuristic — estimated distance from the current node to the
    // end node.
    double m_G, m_H;
    
    double m_depth;
};

/* --------------------------------------------------------------------------
This class actually runs the A* graph search algorithm. There are a multitude
of different options however in the typical use, the constructor is called 
which allows the user to set how how many neighbors to investagate, the depth
that is classified as an obstacle by A*, the start and finish coordinates, and 
a few other options. Then the map is built using a preloaded csv file, a simple
default map of an cross, or from an ENC (STILL NEEDS TO BE DONE). The start
and finish positions can be inputed at any time and are in either the grid 
coordinate system or in local UTM. Then the user can either run A* from a set
of commands:
	-AStar_Search: runs the search algorithm without checking if the 
		waypoints are valid and outputs the A* generated waypoints 
		in a comma seperated string in the grid coordinate system
	-runA_Star: Runs A* after checking the waypoints to see if they are
		valid. Once A* has been run, the function can then be set to
		output different types of files:
			-Print to STD output
			-Output a .bhv MOOS file
			-Output a L84 file
		If the user wants to generate a new MOOS and/or L84 file, a
		base filename must be given.  
--------------------------------------------------------------------------- */
class AStar
{
public:
    // Constuctors/Deconstructor
    AStar(int connectingDistance = 8);
    ~AStar() {}

    // Number of Neighboors one wants to investigate from each cell. A larger
    //    number of nodes means that the path can be alligned in more directions.
    //
    //    Connecting_Distance=1-> Path can  be alligned along 8 different direction.
    //    Connecting_Distance=2-> Path can be alligned along 16 different direction.
    //    Connecting_Distance=3-> Path can be alligned along 32 different direction.
    //    Connecting_Distance=4-> Path can be alligned along 56 different direction.
    //    ETC......
    void NeighborsMask(int connectingDistance);

    //Generate the path from finish to start by following the directions
    //    in the direction map.
    std::vector<Position> reconstructPath(std::map<Position,Node> const &nodeMap, Position const &lastPosition);

    // Check to see if the extened path is valid
    double extendedPathAverageDepth(Context const &c, Position const& position, Position const & newPosition);

    // This function runs A* search. It outputs the generated WPTs as a comma seperated string
    std::vector<Position> search(Context const &c);

    int getNumberDirections() {return m_numberDirections; }
private:
    int m_numberDirections;              // Dimensions (rows,cols) of map, number of directions to search
    std::vector<Position> m_candidates;                    // relative coordinates of candidate nodes from parent
};

bool operator<(Node& lhs, Node& rhs);
bool operator>(Node& lhs, Node& rhs);

} //namespace astar

#endif /* ASTAR_H_ */
