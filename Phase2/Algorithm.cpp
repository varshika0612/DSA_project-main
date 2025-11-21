#include "Algorithm.hpp"
#include <queue>
#include <climits>
#include <algorithm>
#include <unordered_map>
path a_str(int start,int target,const std::set<int>&banned_edges,std::set<int>&banned_nodes){
    path result;
    if (banned_nodes.count(start) || banned_nodes.count(target))return result;
    std::unordered_map<int,double>dist;
    std::unordered_map<int,int> parent;
    std::unordered_map<int,int> parent_edge;
    std::priority_queue<std::pair<double,int>,std::vector<std::pair<double,int>>,std::greater<std::pair<double,int>>> pq;
    dist[start]=0.0;
    pq.push({0.0,start});
    while(!pq.empty()){
        double d=pq.top().first;
        int u=pq.top().second;
        pq.pop();
        if(dist.count(u)&& d>dist[u] )continue;
        if(u==target) break;
        for(int id:graph.getAdjacentEdges(u)){
            if(banned_edges.count(id))continue;
            const Edges*edge=graph.getEdge(id);
            if(!edge || edge->is_removed) continue;
            int v=(edge->u===u)?edge->v:edge->u;
            if(edge->oneway && edge->u!=u)continue;
            if(banned_nodes.count(v))continue;
            double new_dist=d+edge->length;
            if(!dist.count(v)|| new_dist<dist[v]){
                dist[v]=new_dist;
                parent[v]=u;
                parent_edge[v]=id;
                pq.push({new_dist,v});
            }
        }
    }
    if(!dist.count(target))return result;
    result.cost=dist[target];
    int current=target;
    while(current!=start){
        result.nodes.push_back(current);
        if(parent_edge.count(current)){
            result.edges.push_back(parent_edge[current]);
            current=parent[current];
        }
        current=parent[current];
    }
    result.nodes.push_back(start);
    std::reverse(result.nodes.begin(),result.nodes.end());
    std::reverse(result.edges.begin(),result.edges.end());
    return result;
}
std::vector<path> Yen::findK_paths(int start,int target,int k){
    std::vector<path>result;
    std::priority_queue<path,std::vector<path>,std::greater<path>> pq;
    path shortest_path=a_star(start,target,{},{});
    if(shortest_path.nodes.empty())return result;
    result.push_back(shortest_path);
    for(int i=0;i<k;i++){
        path & prev_path=result.back();
        for(size_t j=0;j<prev_path.nodes.size()-1;j++){
            int spur_node=prev_path.nodes[j];
            std::vector<int> root_path(prev_path.nodes.begin(),prev_path.nodes.begin()+i+1);
            std::vector<int> root_edges(prev_path.edges.begin(),prev_path.edges.begin()+i);
            double root_length=0.0;
            for(int edge_id:root_edges){
                const Edge*e=graph->getEdge(edge_id);
                if(e)root_length+=e->length;
            }
            std::set<int> banned_edges;
            std::set<int> banned_nodes;
            
        }
    }
}