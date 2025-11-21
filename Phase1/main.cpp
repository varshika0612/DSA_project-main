#include "Graph.hpp"
#include "Algorithm.hpp"
#include "JsonParser.hpp"
#include "QueryProcessor.hpp"
#include <iostream>
#include <chrono>

int main(int argc, char* argv[]) {
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <graph.json> <queries.json> <output.json>" << std::endl;
        return 1;
    }
    
    std::string graph_file = argv[1];
    std::string queries_file = argv[2];
    std::string output_file = argv[3];

    try {
        // Load graph
        std::cout << "Loading graph from " << graph_file << "..." << std::endl;
        Graph graph = JsonParser::parseGraph(graph_file);
        std::cout << "Graph loaded: " << graph.getNodeCount() << " nodes" << std::endl;
        
        // Load queries
        std::cout << "Loading queries from " << queries_file << "..." << std::endl;
        json queries_json = JsonParser::parseQueries(queries_file);
        
        // Process queries
        QueryProcessor processor(graph);
        json output;
        output["results"] = json::array();
        
        if (queries_json.contains("events")) {
            for (const auto& query : queries_json["events"]) {
                auto start = std::chrono::high_resolution_clock::now();
                
                json result = processor.processQuery(query);
                
                auto end = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
                
                result["processing_time_us"] = duration.count();
                output["results"].push_back(result);
            }
        }
        
        // Write output
        std::cout << "Writing output to " << output_file << "..." << std::endl;
        JsonParser::writeOutput(output_file, output);
        std::cout << "Done!" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}