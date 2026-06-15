/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include "../include/Planner.hpp"
#include <math.h>
#include <stdexcept>
#include <cmath>
#include <iostream>

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

#ifdef PLANNER_ENABLE_DEBUG_LOGGING
#define PLANNER_DEBUG_LOG(MSG) do { std::clog << "[planner] " << MSG << '\n'; } while(false)
#else
#define PLANNER_DEBUG_LOG(MSG) do {} while(false)
#endif
#ifdef PLANNER_ENABLE_LOGGING
#define PLANNER_LOG(MSG) do { std::clog << "[planner] " << MSG << '\n'; } while(false)
#else
#define PLANNER_LOG(MSG) do {} while(false)
#endif

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
        PLANNER_LOG("initialized map=" << m_mapXSize << "x" << m_mapYSize
                    << " nodes=" << m_nodesVector.size()
                    << " collision_threshold=" << m_collisionThreshold
                    << " target_steps=" << m_targetSteps);
    }

float Planner::distanceTo(const Node& a, const Node& b){
    return std::hypotf(a.position.x - b.position.x, a.position.y - b.position.y);
}

float Planner::distanceTo(const Coord& a, const Coord& b){
    return std::hypotf(a.x - b.x, a.y - b.y);
}

Planner::Node& Planner::getNodeFromCoord(const Coord& position){
    if(!mapBoundsCheck(position)){
        PLANNER_LOG("getNodeFromCoord out of bounds pos=(" << position.x << "," << position.y << ")");
        throw std::invalid_argument("getNodeFromCoord() called from out of bounds");
    }
    return m_nodesVector.at(GETMAPINDEX(position.x, position.y, m_mapXSize, m_mapYSize));
}

void Planner::resetSearch(){
    m_nodesVector.assign(m_mapXSize * m_mapYSize, Node{});
    m_openNodes = {};
}

Planner::Node& Planner::createNode(const Coord& position, const Node& goal){
    Node& newNode = m_nodesVector.at(GETMAPINDEX(position.x, position.y, m_mapXSize, m_mapYSize));
    if(newNode.isInitialized == true){
        PLANNER_DEBUG_LOG("reused node pos=(" << position.x << "," << position.y << ")"
                    << " g=" << newNode.g
                    << " h=" << newNode.h
                    << " f=" << newNode.cost());
        return newNode;
    }
    newNode.position.x = position.x;
    newNode.position.y = position.y;
    newNode.h = distanceTo(newNode, goal);
    newNode.isInitialized = true;
    PLANNER_DEBUG_LOG("created node pos=(" << position.x << "," << position.y << ")"
                << " g=" << newNode.g
                << " h=" << newNode.h
                << " f=" << newNode.cost());
    return newNode;
}

Planner::Node& Planner::createNode(const Coord& position, const Coord& goal){
    Node& newNode = m_nodesVector.at(GETMAPINDEX(position.x, position.y, m_mapXSize, m_mapYSize));
    if(newNode.isInitialized == true){
        PLANNER_DEBUG_LOG("reused node pos=(" << position.x << "," << position.y << ")"
                    << " g=" << newNode.g
                    << " h=" << newNode.h
                    << " f=" << newNode.cost());
        return newNode;
    }
    newNode.position.x = position.x;
    newNode.position.y = position.y;
    newNode.h = distanceTo(position, goal);
    newNode.isInitialized = true;
    PLANNER_DEBUG_LOG("created node pos=(" << position.x << "," << position.y << ")"
                << " g=" << newNode.g
                << " h=" << newNode.h
                << " f=" << newNode.cost());
    return newNode;
}

bool Planner::mapBoundsCheck(const Coord& position){
    return (position.x >= 1) && (position.x <= m_mapXSize) && (position.y >= 1) && (position.y <= m_mapYSize);
}

bool Planner::nodeCostCheck(const Coord& position){
    if(!mapBoundsCheck(position)){
        PLANNER_LOG("nodeCostCheck out of bounds pos=(" << position.x << "," << position.y << ")");
        throw std::invalid_argument("nodeCostCheck() called from out of bounds");
    }
    return m_map[GETMAPINDEX(position.x, position.y, m_mapXSize, m_mapYSize)] < m_collisionThreshold;
}

void Planner::exploreOpenNode(const Coord& goalPose){
    Node currentNode = m_openNodes.top();
    m_openNodes.pop(); // remove this node as it is explored.
    PLANNER_DEBUG_LOG("expand pos=(" << currentNode.position.x << "," << currentNode.position.y << ")"
                << " g=" << currentNode.g
                << " h=" << currentNode.h
                << " f=" << currentNode.cost()
                << " open_remaining=" << m_openNodes.size());
    for(size_t index = 0; index < deltaX.size(); index++){

        Coord neighborPos{currentNode.position.x + deltaX[index],
                           currentNode.position.y + deltaY[index]};

        if (!mapBoundsCheck(neighborPos)){
            PLANNER_DEBUG_LOG("  skip neighbor pos=(" << neighborPos.x << "," << neighborPos.y << ") reason=out_of_bounds");
            continue;
        }
        if (!nodeCostCheck(neighborPos)){
            PLANNER_DEBUG_LOG("  skip neighbor pos=(" << neighborPos.x << "," << neighborPos.y << ") reason=collision"
                        << " map_cost=" << m_map[GETMAPINDEX(neighborPos.x, neighborPos.y, m_mapXSize, m_mapYSize)]);
            continue;
        }

        Node& neighborNode = createNode(neighborPos, goalPose);
        float candidateG = currentNode.g + 1;
        if (neighborNode.g > candidateG){
            PLANNER_DEBUG_LOG("  improve neighbor pos=(" << neighborPos.x << "," << neighborPos.y << ")"
                        << " old_g=" << neighborNode.g
                        << " new_g=" << candidateG
                        << " parent=(" << currentNode.position.x << "," << currentNode.position.y << ")");
            neighborNode.parent = currentNode.position;
            neighborNode.g = candidateG;
            m_openNodes.push(neighborNode);
            PLANNER_DEBUG_LOG("  push neighbor pos=(" << neighborPos.x << "," << neighborPos.y << ")"
                        << " f=" << neighborNode.cost()
                        << " open_size=" << m_openNodes.size());
        } else {
            PLANNER_DEBUG_LOG("  keep neighbor pos=(" << neighborPos.x << "," << neighborPos.y << ")"
                        << " current_g=" << neighborNode.g
                        << " candidate_g=" << candidateG);
        }
    }
}

Planner::Coord Planner::returnBestNextStep(const Node& start, const Node& goal){
    Node currentNode = createNode(goal.position, goal);
    PLANNER_LOG("backtrack from goal=(" << goal.position.x << "," << goal.position.y << ")"
                << " first_parent=(" << goal.parent.x << "," << goal.parent.y << ")");
    while(currentNode.parent != start.position){

        PLANNER_DEBUG_LOG("  backtrack node=(" << currentNode.position.x << "," << currentNode.position.y << ")"
                    << " parent=(" << currentNode.parent.x << "," << currentNode.parent.y << ")");
        currentNode = createNode(currentNode.parent, goal);
    }
    PLANNER_LOG("next step=(" << currentNode.position.x << "," << currentNode.position.y << ")");
    return currentNode.position;

}

Planner::Coord Planner::computeGoalCoordinate(int curr_time, int lookAhead){
    Coord goalCoordinate;
    int mapIndex = std::min(m_targetSteps-1, (curr_time+lookAhead));
    goalCoordinate.x = m_targetTraj[mapIndex];
    goalCoordinate.y = m_targetTraj[mapIndex + m_targetSteps];
    PLANNER_LOG("goal lookup curr_time=" << curr_time
                << " look_ahead=" << lookAhead
                << " target_index=" << mapIndex
                << " goal=(" << goalCoordinate.x << "," << goalCoordinate.y << ")");
    return goalCoordinate;
}

void Planner::ComputeAction(
    int robotposeX,
    int robotposeY,
    int targetposeX,
    int targetposeY,
    int curr_time,
    int* action_ptr,
    int lookAhead)
{
    resetSearch();
    Coord goalPose = computeGoalCoordinate(curr_time, lookAhead);
    PLANNER_LOG("ComputeAction start robot=(" << robotposeX << "," << robotposeY << ")"
                << " target=(" << targetposeX << "," << targetposeY << ")"
                << " goal=(" << goalPose.x << "," << goalPose.y << ")"
                << " curr_time=" << curr_time);
    Node& goal = createNode(goalPose, goalPose);
    Node& start = createNode(Coord{robotposeX, robotposeY}, goal);
    start.g = 0;
    PLANNER_LOG("start node g set to 0 pos=(" << start.position.x << "," << start.position.y << ")");

    m_openNodes.push(start);
    PLANNER_LOG("push start open_size=" << m_openNodes.size());
    while(m_openNodes.top() != goal){
        exploreOpenNode(goalPose);
    }
    PLANNER_LOG("goal reached top=(" << m_openNodes.top().position.x << "," << m_openNodes.top().position.y << ")"
                << " open_size=" << m_openNodes.size());

    Coord nextPosition = returnBestNextStep(start, goal);
    robotposeX = nextPosition.x;
    robotposeY = nextPosition.y;
    action_ptr[0] = robotposeX;
    action_ptr[1] = robotposeY;
    PLANNER_LOG("selected action=(" << action_ptr[0] << "," << action_ptr[1] << ")");
    
    return;
}
