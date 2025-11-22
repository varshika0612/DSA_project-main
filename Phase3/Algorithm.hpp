#ifndef ALGO_HPP
#define ALGO_HPP

#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <limits>
#include <algorithm>
#include <numeric>
#include <unordered_map>
#include <set>
#include <tuple>

#include "../Phase2/Algorithm.hpp"
#include "Graph.hpp"

constexpr double INF = std::numeric_limits<double>::infinity();

// --- GraphAdapter: Bridge between scheduler and Algorithm.cpp ---
class GraphAdapter {
private:
    const Graph& lib_graph;

    struct PairHash {
        size_t operator()(const std::pair<int,int>& p) const noexcept {
            size_t h1 = std::hash<int>()(p.first);
            size_t h2 = std::hash<int>()(p.second);
            return h1 ^ (h2 << 1);
        }
    };

    std::unordered_map<std::pair<int,int>, double, PairHash> distance_cache;

public:
    explicit GraphAdapter(const Graph& g);
    
    double getApproximateDistance(int u, int v);
    double getExactShortestDistance(int u, int v);
    std::vector<path> getKShortestPaths(int u, int v, int k);
    void precomputeLandmarks();
    const Graph& getLibGraph() const;
};

// --- Order ---
struct Order {
    int id;
    int pickup_node;
    int dropoff_node;
    double completion_time = 0.0;
};

// --- RouteStop ---
struct RouteStop {
    int node_id;
    int order_id;
    bool is_pickup;
    double arrival_time;
};

// --- Driver ---
class Driver {
public:
    int id;
    std::vector<RouteStop> route;
    double current_time;
    std::set<int> orders_being_carried;

    Driver(int driver_id, int depot_node);
    
    void recalculateRouteTimes(GraphAdapter& adapter, bool use_approximate = false);
    double findBestInsertion(const Order& new_order, GraphAdapter& adapter, 
                            int& best_p_idx, int& best_d_idx,
                            bool use_approximate = true) const;
    void commitInsertion(const Order& new_order, int p_idx, int d_idx);
    std::vector<int> getFinalRouteNodes() const;
};

// --- Scheduler ---
class Scheduler {
private:
    GraphAdapter& adapter;
    int depot_node;
    std::vector<Driver> drivers;
    std::vector<Order> orders;
    bool use_approximate_distances;

public:
    struct AssignmentOutput {
        int driver_id;
        std::vector<int> route;
        std::vector<int> order_ids;
    };
    
    struct MetricsOutput {
        double total_delivery_time_s;
        double max_delivery_time_s;
    };

    struct Output {
        std::vector<AssignmentOutput> assignments;
        MetricsOutput metrics;
    };

    Scheduler(GraphAdapter& ga, int depot, int num_drivers, bool use_approx = true);
    
    void loadOrders(std::vector<Order>&& o);
    void findAlternativeRoutes(int pickup, int dropoff, int k = 3);
    void run();
    Output getResults();
};

#endif // ALGO_HPP