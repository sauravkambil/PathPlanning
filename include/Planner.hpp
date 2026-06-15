#ifndef PLANNER_HPP
#define PLANNER_HPP

#include <vector>
#include <queue>
#include <limits>
class Planner{
public:
    Planner(int* map, int collision_thresh, int x_size, int y_size, int* target_traj, int target_steps);
    void ComputeAction(int robotposeX,
                        int robotposeY,
                        int targetposeX,
                        int targetposeY,
                        int curr_time,
                        int* action_ptr,
                        int lookAhead = 0);
private:
    struct Coord{
        int x{};
        int y{};
        bool operator==(const Coord& other) const = default;
    };
    
    struct Node{
        float g{std::numeric_limits<float>::max()};
        float h{};
        float cost() const{
            return g + h;
        }
        Coord position;
        bool visited{false};
        Coord parent;
        bool isInitialized{false};
        bool operator==(const Node& node) const {
            return position == node.position;
        }
    };

    int* m_map;
    int m_collisionThreshold;
    int m_mapXSize;
    int m_mapYSize;
    int* m_targetTraj;
    int m_targetSteps;
    std::vector<int> deltaX;
    std::vector<int> deltaY;

    struct LowerPriority {
        bool operator()(const Node& a, const Node& b) const {
            if (a.cost() == b.cost()) {
                return a.h > b.h;
            }
            return a.cost() > b.cost();
        }
    };

    std::priority_queue<Node, std::vector<Node>, LowerPriority> m_openNodes{};

    std::vector<Node> m_nodesVector{};

    void exploreOpenNode(const Coord& goalPose);

    //helpers
    float distanceTo(const Node& a, const Node&b);
    float distanceTo(const Coord& a, const Coord&b);
    Node& createNode(const Coord& position, const Node& goal);
    Node& createNode(const Coord& position, const Coord& goal);
    bool mapBoundsCheck(const Coord& position);
    bool nodeCostCheck(const Coord& position);
    Node& getNodeFromCoord(const Coord& position);
    Coord returnBestNextStep(const Node& start, const Node& goal);
    Coord computeGoalCoordinate(int curr_time, int lookAhead);
    void resetSearch();
};
#endif // PLANNER_HPP