/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include "../include/Planner.hpp"
#include <math.h>
#include <stdexcept>
#include <cmath>

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

Planner::Planner(int* map, int collision_thresh, int x_size, int y_size, int* target_traj, int target_steps):m_map(map),
    m_collisionThreshold(collision_thresh),
    m_mapXSize(x_size),
    m_mapYSize(y_size),
    m_targetTraj(target_traj),
    m_targetSteps(target_steps)
    {
        if(map == nullptr || target_traj == nullptr){
            throw std::invalid_argument("map cannot be a null argument");
        }
        deltaX = {-1, -1, -1,  0,  0,  1, 1, 1};
        deltaY = {-1,  0,  1, -1,  1, -1, 0, 1};
        m_nodesVector.resize(m_mapXSize*m_mapYSize);
    }

float Planner::distanceTo(const Node& a, const Node& b){
    return std::hypotf(a.position.x - b.position.x, a.position.y - b.position.y);
}

float Planner::distanceTo(const Coord& a, const Coord& b){
    return std::hypotf(a.x - b.x, a.y - b.y);
}

Planner::Node& Planner::getNodeFromCoord(const Coord& position){
    if(!mapBoundsCheck(position)){
        throw std::invalid_argument("getNodeFromCoord() called from out of bounds");
    }
    return m_nodesVector.at(GETMAPINDEX(position.x, position.y, m_mapXSize, m_mapYSize));
}

Planner::Node& Planner::createNode(const Coord& position, const Node& goal){
    Node& newNode = m_nodesVector.at(GETMAPINDEX(position.x, position.y, m_mapXSize, m_mapYSize));
    newNode.position.x = position.x;
    newNode.position.y = position.y;
    newNode.h = distanceTo(newNode, goal);
    newNode.isInitialized = true;
    return newNode;
}

Planner::Node& Planner::createNode(const Coord& position, const Coord& goal){
    Node& newNode = m_nodesVector.at(GETMAPINDEX(position.x, position.y, m_mapXSize, m_mapYSize));
    if(newNode.isInitialized == true){
        return newNode;
    }
    newNode.position.x = position.x;
    newNode.position.y = position.y;
    newNode.h = distanceTo(position, goal);
    newNode.isInitialized = true;
    return newNode;
}

bool Planner::mapBoundsCheck(const Coord& position){
    return (position.x >= 1) && (position.x <= m_mapXSize) && (position.y >= 1) && (position.y <= m_mapYSize);
}

bool Planner::nodeCostCheck(const Coord& position){
    if(!mapBoundsCheck(position)){
        throw std::invalid_argument("nodeCostCheck() called from out of bounds");
    }
    return m_map[GETMAPINDEX(position.x, position.y, m_mapXSize, m_mapYSize)] < m_collisionThreshold;
}

void Planner::exploreOpenNode(const Coord& goalPose){
    Node currentNode = m_openNodes.top();
    m_openNodes.pop(); // remove this node as it is explored.
    for(size_t index = 0; index < deltaX.size(); index++){

        Coord neighborPos{currentNode.position.x + deltaX[index],
                           currentNode.position.y + deltaY[index]};

        if (mapBoundsCheck(neighborPos) && nodeCostCheck(neighborPos)){
            Node& neighborNode = createNode(neighborPos, goalPose);
            if (neighborNode.g > currentNode.g + 1){
                neighborNode.parent = currentNode.position;
                neighborNode.g = currentNode.g + 1;
                m_openNodes.push(neighborNode);
            }             
        }
    }
}

Planner::Coord Planner::returnBestNextStep(const Node& start, const Node& goal){
    Node currentNode = createNode(goal.parent, goal);
    while(currentNode.parent != start.position){
        currentNode = createNode(currentNode.parent, goal);
    }
    return currentNode.position;

}

void Planner::ComputeAction(
    int robotposeX,
    int robotposeY,
    int targetposeX,
    int targetposeY,
    int curr_time,
    int* action_ptr)
{
    Coord goalPose{};
    goalPose.x = m_targetTraj[m_targetSteps-1];
    goalPose.y = m_targetTraj[m_targetSteps-1+m_targetSteps];
    // printf("robot: %d %d;\n", robotposeX, robotposeY);
    // printf("goal: %d %d;\n", goalposeX, goalposeY);
    Node& goal = createNode(goalPose, goalPose);
    Node& start = createNode(Coord{robotposeX, robotposeY}, goal);
    start.g = 0;

    m_openNodes.push(start);
    while(m_openNodes.top() != goal){
        exploreOpenNode(goalPose);
    }

    Coord nextPosition = returnBestNextStep(start, goal);
    robotposeX = nextPosition.x;
    robotposeY = nextPosition.y;
    action_ptr[0] = robotposeX;
    action_ptr[1] = robotposeY;
    
    return;
}