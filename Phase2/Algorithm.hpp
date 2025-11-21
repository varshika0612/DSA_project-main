#include "Graph.hpp"
#include<queue>
#include<set>
#include <algorithm>
struct path{
    std::vector<int> edges;
    std::vector<int> nodes;
    double cost;
    bool operator >(const path&other) const{
        return cost>other.cost;
    }
};
class Yen{
    private:
    const Graph*graph;
    std::unordered_map<int, double> heuristic_map;
    std::unordered_map<int, std::vector<int>> reverse_adj_list;
    void buildReverseGraph();
    void computeHeuristics(int target_id);
    struct AStarNode {
        int id;
        double g_score; 
        double f_score; 
        int parent_node;
        int parent_edge;

        bool operator>(const AStarNode& other) const {
            return f_score > other.f_score;
        }
    };
    path a_star(int start,int end,const std::set<int>&banned_edges,const std::set<int>&banned_nodes);
    public:
    Yen(const Graph*g):graph(g){}
    std::vector<path> findK_paths(int start,int end,int k);

};