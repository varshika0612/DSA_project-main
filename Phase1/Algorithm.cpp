#include "Algorithm.hpp"
#include <queue>
#include <climits>
#include <algorithm>
#include <unordered_map>

PathResult Algorithms:: shortestPathDistance(
        const Graph&graph,
        int src,
        int dst,
        const Constraints&constrains
    ){
        //initiate
        PathResult result;
        result.possible=false;
        result.cost=INF;
        if(constrains.forbidden_nodes.count(src)|| constrains.forbidden_nodes.count(dst) ){
            return result;
        }
        const Node* src_node=graph.getNode(src);
        const Node * dst_node=graph.getNode(dst);
        if(!src_node || !dst_node) return result;
        std::unordered_map<int,double>dist;
        std::unordered_map<int,int> parent;
        std::priority_queue<std::pair<double,int>,std::vector<std::pair<double,int>>,std::greater<>> pq;
         dist[src]=0.0;//push src,and make its distance 0;
        double d_start=graph.euclideanDistance(src_node->lat,src_node->lon,dst_node->lat,dst_node->lon);
        pq.push({0.0+d_start,src});
         while(!pq.empty()){
            double d=pq.top().first;
            int u=pq.top().second;
            pq.pop();
            if(u==dst) break;
            if(dist.count(u) && d > dist[u]) continue;
            for(int edge_id:graph.getAdjacentEdges(u)){
                const Edge *e=graph.getEdge(edge_id);
                if(!e || e->is_removed)continue;
                if(!constrains.forbidden_road_types.empty() && constrains.forbidden_road_types.count(e->road_type))continue;
                int v=(e->u==u)?e->v:e->u;
                if(e->oneway && e->v !=v)continue;
                if(constrains.forbidden_nodes.count(v))continue;
                double new_dist=dist[u]+e->length;
                if(!dist.count(v) || new_dist<dist[v]){
                    dist[v]=new_dist;
                    parent[v]=u;
                    const Node * n_v=graph.getNode(v);
                    double h=graph.euclideanDistance(n_v->lat,n_v->lon,dst_node->lat,dst_node->lon);
                    pq.push({new_dist+h,v});

                }
            }
        }
            if(!dist.count(dst))return result;
            if(dst!=src && !parent.count(dst))return result;
            std::vector<int> path;
            int cur=dst;
            while(cur!=src){
                if(!parent.count(cur))return result;
                path.push_back(cur);
                cur=parent[cur];
            }
            path.push_back(src);
            std::reverse(path.begin(),path.end());
            result.possible=true;
            result.cost=dist[dst];
            result.path=path;
            return result;
        }


    PathResult Algorithms:: shortestPathTime(const Graph&graph,int src,int dst,const Constraints&constrains){
        //initiate
        PathResult result;
        result.possible=false;
        result.cost=INF;
        if(constrains.forbidden_nodes.count(src)|| constrains.forbidden_nodes.count(dst) ){
            return result;
        }
        const Node* src_node=graph.getNode(src);
        const Node * dst_node=graph.getNode(dst);
        if(!src_node || !dst_node) return result;
        std::unordered_map<int,double>time;
        std::unordered_map<int,int> parent;
        std::priority_queue<std::pair<double,int>,std::vector<std::pair<double,int>>,std::greater<>> pq;
         time[src]=0.0;//push src,and make its distance 0;
        double d_start=graph.euclideanDistance(src_node->lat,src_node->lon,dst_node->lat,dst_node->lon);
        pq.push({0.0+d_start,src});
         while(!pq.empty()){
            double d=pq.top().first;
            int u=pq.top().second;
            pq.pop();
            if(u==dst) break;
            double u_to_dst=graph.euclideanDistance(graph.getNode(u)->lat,graph.getNode(u)->lon,dst_node->lat,dst_node->lon);
            if(time.count(u) && d-u_to_dst>time[u])continue;
            for(int edge_id:graph.getAdjacentEdges(u)){
                const Edge *e=graph.getEdge(edge_id);

                if(!e || e->is_removed)continue;

                if(!constrains.forbidden_road_types.empty() && constrains.forbidden_road_types.count(e->road_type))continue;
               
                int v=(e->u==u)?e->v:e->u;
                if(e->oneway && e->v !=v)continue;
                if(constrains.forbidden_nodes.count(v))continue;

                double edge_time = graph.getEdgeTime(edge_id, time[u]);
                double new_time = time[u] + edge_time;
                if(!time.count(v) || new_time<time[v]){
                    time[v]=new_time;
                    parent[v]=u;
                    const Node * n_v=graph.getNode(v);
                    double h=graph.euclideanDistance(n_v->lat,n_v->lon,dst_node->lat,dst_node->lon);
                    pq.push({new_time+h,v});

                }
            }
        }
            if(!time.count(dst))return result;
            if(dst!=src && !parent.count(dst))return result;
            std::vector<int> path;
            int cur=dst;
            while(cur!=src){
                if(!parent.count(cur))return result;
                path.push_back(cur);
                cur=parent[cur];
            }
            path.push_back(src);
            std::reverse(path.begin(),path.end());
            result.possible=true;
            result.cost=time[dst];
            result.path=path;
            return result;
        }
    int Algorithms::findNearestNode(const Graph&graph, double lat,double lon){
        int nearest=-1;
        double min_dist=INT_MAX;
        for(const Node &node:graph.getNodes()){
            double dist=graph.euclideanDistance(lat,lon,node.lat,node.lon);
            if(dist<min_dist){
                min_dist=dist;
                nearest=node.id;
            }
        }
            return nearest;

    }
std::vector<int> Algorithms::knnEuclidean(const Graph &graph,double query_lat,double query_lon,const std::string&poi_type,int k){
        std::priority_queue<std::pair<double,int>> distances;
        int cur_size=0;
        for(const Node &node:graph.getNodes()){
            for(const std::string&poi:node.pois){
                if(poi==poi_type){
                 double dist=graph.euclideanDistance(query_lat,query_lon,node.lat,node.lon);
                 if(cur_size<k){
                    distances.push({dist,node.id});
                    cur_size++;
                 }
                 else if(dist<distances.top().first){
                    distances.pop();
                    distances.push({dist,node.id});
                 }
                    break;
                }
            }
            }
            std::vector<int> result(cur_size);
            for(int i=0;i<cur_size;i++){
                result[cur_size-i-1]=distances.top().second;
                distances.pop();
            }
            return result;
    }

 std::vector<int> Algorithms::knnShortestPath(const Graph& graph, double query_lat, double query_lon, const std::string& poi_type, int k) {
    // 1. Find start node
    int start_node = findNearestNode(graph, query_lat, query_lon);
    if (start_node == -1) return {};

    // 2. Setup Dijkstra
    // Initialize with Infinity
    std::vector<double> dist(graph.getNodes().size(), std::numeric_limits<double>::max());
    
    std::priority_queue<std::pair<double, int>, 
                        std::vector<std::pair<double, int>>, 
                        std::greater<std::pair<double, int>>> pq;

    dist[start_node] = 0.0;
    pq.push({0.0, start_node});

    std::vector<int> result;

    // 3. Run Dijkstra with Early Exit
    while (!pq.empty()) {
        double d = pq.top().first;
        int u = pq.top().second;
        pq.pop();

        // Standard Dijkstra check: if we found a shorter way before, skip
        if (d > dist[u]) continue;

        // --- OPTIMIZATION START ---
        // Check if current node 'u' is the POI we need.
        // Since Dijkstra visits nodes in strictly increasing order of distance,
        // the first 'k' POIs we find are GUARANTEED to be the closest 'k'.
        const Node& u_node = graph.getNodes()[u]; // Assuming efficient access
        bool is_target_poi = false;
        
        // Check the POI string vector
        for (const auto& type : u_node.pois) {
            if (type == poi_type) {
                is_target_poi = true;
                break;
            }
        }

        if (is_target_poi) {
            result.push_back(u);
            // If we have found k items, WE STOP IMMEDIATELY.
            // No need to search the rest of the graph.
            if (result.size() == k) {
                return result;
            }
        }

        // Expand neighbors
        for (int edge_id : graph.getAdjacentEdges(u)) {
            const Edge* e = graph.getEdge(edge_id);
            if (!e || e->is_removed) continue;

            int v = (e->u == u) ? e->v : e->u;
            if (e->oneway && e->v != v) continue;

            double new_dist = dist[u] + e->length;

            if (new_dist < dist[v]) {
                dist[v] = new_dist;
                pq.push({new_dist, v});
            }
        }
    }

    return result;
}