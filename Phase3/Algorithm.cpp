#include "Algorithm.hpp"

// ==========================================
//          GraphAdapter Implementation
// ==========================================

GraphAdapter::GraphAdapter(const Graph& g) : lib_graph(g) {}

double GraphAdapter::getApproximateDistance(int u, int v) {
    if (u == v) return 0.0;
    return Algorithm::approximateShortestPathDistance(lib_graph, u, v);
}

double GraphAdapter::getExactShortestDistance(int u, int v) {
    if (u == v) return 0.0;
    
    auto key = std::make_pair(u, v);
    auto it = distance_cache.find(key);
    if (it != distance_cache.end()) return it->second;

    // Use Yen's A* for exact shortest path
    // Yen yen_solver;
    // yen_solver.setGraph(const_cast<Graph*>(&lib_graph));
    auto paths = Algorithm::kShortestPathsHeuristic(lib_graph, u, v, 1, 0.5);
        
        double dist = paths.empty() ? INF : paths[0].cost;
    distance_cache[key] = dist;
    return dist;
}

std::vector<path> GraphAdapter::getKShortestPaths(int u, int v, int k) {
    return Algorithm::kShortestPathsHeuristic(lib_graph, u, v, k, 0.5);
}

void GraphAdapter::precomputeLandmarks() {
    Algorithm::precomputeLandmarks(lib_graph);
}

const Graph& GraphAdapter::getLibGraph() const { 
    return lib_graph; 
}

// ==========================================
//          Driver Implementation
// ==========================================

Driver::Driver(int driver_id, int depot_node) 
    : id(driver_id), current_time(0.0) {
    route.push_back({depot_node, -1, false, 0.0});
}

void Driver::recalculateRouteTimes(GraphAdapter& adapter, bool use_approximate) {
    if (route.empty()) {
        current_time = 0.0;
        return;
    }

    route[0].arrival_time = 0.0;
    std::unordered_map<int, double> pickup_times;
    
    for (size_t i = 1; i < route.size(); ++i) {
        int u = route[i - 1].node_id;
        int v = route[i].node_id;
        
        double travel_time = use_approximate 
            ? adapter.getApproximateDistance(u, v)
            : adapter.getExactShortestDistance(u, v);
        
        double arrival = route[i - 1].arrival_time + travel_time;
        
        if (!route[i].is_pickup) {
            int order_id = route[i].order_id;
            if (pickup_times.count(order_id)) {
                arrival = std::max(arrival, pickup_times.at(order_id));
            }
        }
        
        route[i].arrival_time = arrival;
        
        if (route[i].is_pickup) {
            pickup_times[route[i].order_id] = route[i].arrival_time;
        }
    }
    
    current_time = route.back().arrival_time;
}

double Driver::findBestInsertion(const Order& new_order, GraphAdapter& adapter, 
                                int& best_p_idx, int& best_d_idx,
                                bool use_approximate) const {
    const int L = route.size();
    double min_new_time = INF;
    
    for (int p_idx = 0; p_idx <= L; ++p_idx) {
        for (int d_idx = p_idx + 1; d_idx <= L + 1; ++d_idx) {
            std::vector<RouteStop> temp_route = route;
            
            temp_route.insert(temp_route.begin() + p_idx, 
                {new_order.pickup_node, new_order.id, true, 0.0});
            
            int actual_d_idx = d_idx;
            temp_route.insert(temp_route.begin() + actual_d_idx, 
                {new_order.dropoff_node, new_order.id, false, 0.0});

            std::unordered_map<int, double> pickup_times;
            temp_route[0].arrival_time = 0.0;

            for (size_t i = 1; i < temp_route.size(); ++i) {
                int u = temp_route[i - 1].node_id;
                int v = temp_route[i].node_id;
                
                double travel_time = use_approximate
                    ? adapter.getApproximateDistance(u, v)
                    : adapter.getExactShortestDistance(u, v);
                    
                if (travel_time == INF) continue;
                
                double arrival = temp_route[i - 1].arrival_time + travel_time;
                
                if (!temp_route[i].is_pickup) {
                    int order_id = temp_route[i].order_id;
                    if (pickup_times.count(order_id)) {
                        arrival = std::max(arrival, pickup_times.at(order_id));
                    }
                }
                
                temp_route[i].arrival_time = arrival;
                
                if (temp_route[i].is_pickup) {
                    pickup_times[temp_route[i].order_id] = temp_route[i].arrival_time;
                }
            }
            
            double new_total_time = temp_route.back().arrival_time;

            if (new_total_time < min_new_time) {
                min_new_time = new_total_time;
                best_p_idx = p_idx;
                best_d_idx = actual_d_idx;
            }
        }
    }

    return min_new_time;
}

void Driver::commitInsertion(const Order& new_order, int p_idx, int d_idx) {
    route.insert(route.begin() + d_idx, 
        {new_order.dropoff_node, new_order.id, false, 0.0});
    route.insert(route.begin() + p_idx, 
        {new_order.pickup_node, new_order.id, true, 0.0});
    orders_being_carried.insert(new_order.id);
}

std::vector<int> Driver::getFinalRouteNodes() const {
    std::vector<int> nodes;
    nodes.reserve(route.size());
    for (const auto& stop : route) {
        nodes.push_back(stop.node_id);
    }
    return nodes;
}

// ==========================================
//          Scheduler Implementation
// ==========================================

Scheduler::Scheduler(GraphAdapter& ga, int depot, int num_drivers, bool use_approx) 
    : adapter(ga), depot_node(depot), use_approximate_distances(use_approx) {
    drivers.reserve(num_drivers);
    for (int i = 0; i < num_drivers; ++i) {
        drivers.emplace_back(i, depot_node);
    }
}

void Scheduler::loadOrders(std::vector<Order>&& o) {
    orders = std::move(o);
    
    std::cout << "Precomputing landmarks for approximate shortest paths..." << std::endl;
    adapter.precomputeLandmarks();
    std::cout << "Landmark precomputation complete." << std::endl;
}

void Scheduler::findAlternativeRoutes(int pickup, int dropoff, int k) {
    std::cout << "\nFinding " << k << " alternative routes from " 
              << pickup << " to " << dropoff << ":" << std::endl;
    
    auto paths = adapter.getKShortestPaths(pickup, dropoff, k);
    
    for (size_t i = 0; i < paths.size(); ++i) {
        std::cout << "  Route " << (i+1) << ": cost=" << paths[i].cost 
                  << ", nodes=[";
        for (size_t j = 0; j < paths[i].nodes.size(); ++j) {
            std::cout << paths[i].nodes[j];
            if (j < paths[i].nodes.size() - 1) std::cout << "->";
        }
        std::cout << "]" << std::endl;
    }
}

void Scheduler::run() {
    std::cout << "Starting Enhanced Scheduler with Algorithm.cpp integration..." << std::endl;
    
    using DriverLoad = std::pair<double, int>;
    std::priority_queue<DriverLoad, std::vector<DriverLoad>, std::greater<DriverLoad>> driver_heap;

    for (const auto& driver : drivers) {
        driver_heap.push({driver.current_time, driver.id});
    }

    for (size_t i = 0; i < orders.size(); ++i) {
        Order& current_order = orders[i];

        DriverLoad top_driver_load = driver_heap.top();
        driver_heap.pop();
        
        int driver_id = top_driver_load.second;
        Driver& assigned_driver = drivers[driver_id];
        
        int best_p_idx = -1;
        int best_d_idx = -1;
        
        double new_time = assigned_driver.findBestInsertion(
            current_order, adapter, best_p_idx, best_d_idx, use_approximate_distances);

        if (new_time == INF) {
            std::cerr << "Error: No valid path for Order " << current_order.id << std::endl;
            driver_heap.push({assigned_driver.current_time, assigned_driver.id});
            continue;
        }
        
        assigned_driver.commitInsertion(current_order, best_p_idx, best_d_idx);
        assigned_driver.recalculateRouteTimes(adapter, false);
        
        auto it = std::find_if(assigned_driver.route.rbegin(), assigned_driver.route.rend(), 
                               [&current_order](const RouteStop& s){
                                   return s.order_id == current_order.id && !s.is_pickup;
                               });
        if (it != assigned_driver.route.rend()) {
            current_order.completion_time = it->arrival_time;
        }

        driver_heap.push({assigned_driver.current_time, assigned_driver.id});
    }
}

Scheduler::Output Scheduler::getResults() {
    Output result;
    double total_delivery_time = 0.0;
    double max_delivery_time = 0.0;
    
    for (const auto& order : orders) {
        total_delivery_time += order.completion_time;
        max_delivery_time = std::max(max_delivery_time, order.completion_time);
    }
    
    result.metrics.total_delivery_time_s = total_delivery_time;
    result.metrics.max_delivery_time_s = max_delivery_time;
    
    for (const auto& driver : drivers) {
        AssignmentOutput assignment;
        assignment.driver_id = driver.id;
        assignment.route = driver.getFinalRouteNodes();
        
        std::set<int> unique_order_ids;
        for(const auto& stop : driver.route) {
            if (stop.order_id != -1) {
                unique_order_ids.insert(stop.order_id);
            }
        }
        assignment.order_ids.assign(unique_order_ids.begin(), unique_order_ids.end());
        result.assignments.push_back(assignment);
    }
    
    return result;
}