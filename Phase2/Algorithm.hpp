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
    path a_star(int start,int end,const std::set<int>&banned_edges,const std::set<int>&banned_nodes);
    public:
    Yen(const Graph*g):graph(g){}
    std::vector<path> findK_paths(int start,int end,int k);

};