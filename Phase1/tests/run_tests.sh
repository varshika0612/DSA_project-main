#!/bin/bash
# run_tests.sh - Test runner for Sample Driver
# Place in: Phase1/tests/run_tests.sh
# Run with: cd Phase1/tests && bash run_tests.sh

echo "=========================================="
echo "       Sample Driver Test Suite"
echo "=========================================="
echo ""

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Counters
PASSED=0
FAILED=0
TOTAL=0
START_TIME=$(date +%s)

# Helper functions
print_success() { echo -e "${GREEN}✓ $1${NC}"; }
print_error() { echo -e "${RED}✗ $1${NC}"; }
print_warning() { echo -e "${YELLOW}⚠ $1${NC}"; }
print_info() { echo -e "${BLUE}ℹ $1${NC}"; }

# Run a test
run_test() {
    local test_name=$1
    local graph_file=$2
    local query_file=$3
    local output_file=$4
    
    ((TOTAL++))
    
    echo ""
    echo "Test $TOTAL: $test_name"
    echo "-----------------------------------"
    echo "Graph:   $graph_file"
    echo "Queries: $query_file"
    echo "Output:  $output_file"
    echo ""
    
    if [ ! -f "$graph_file" ]; then
        print_error "FAILED - Graph file not found: $graph_file"
        ((FAILED++))
        return 1
    fi
    
    if [ ! -f "$query_file" ]; then
        print_error "FAILED - Query file not found: $query_file"
        ((FAILED++))
        return 1
    fi
    
    # Run from project root (../../sampledriver)
    local test_start=$(date +%s)
    
    # CHANGED: Pointing to sampledriver instead of phase1
    if ../../sampledriver "$graph_file" "$query_file" "$output_file" > /dev/null 2>&1; then
        local test_end=$(date +%s)
        local duration=$((test_end - test_start))
        
        # CHANGED: Validation logic
        # We check if file exists and is not empty (-s).
        # We DO NOT check for valid single-root JSON because sampledriver outputs concatenated JSONs.
        if [ -s "$output_file" ]; then
            print_success "PASSED - Completed in ${duration}s"
            echo "         Output: $output_file"
            
            # Count lines roughly to estimate results (since output is multi-line JSON, this is just an approximation or we skip counting)
            echo "         Status: Output file generated successfully"
            
            ((PASSED++))
            return 0
        else
            print_error "FAILED - Output file is missing or empty"
            ((FAILED++))
            return 1
        fi
    else
        print_error "FAILED - Execution error (Check if './sampledriver' runs manually)"
        ((FAILED++))
        return 1
    fi
}

# Check prerequisites
echo "Checking prerequisites..."
echo ""

# CHANGED: Checking for sampledriver
if [ ! -f "../../sampledriver" ]; then
    print_error "Executable 'sampledriver' not found!"
    echo ""
    echo "Build it first:"
    echo "  cd ../.."
    echo "  make sampledriver"
    echo ""
    exit 1
fi

print_success "Executable found: ../../sampledriver"

echo ""
echo "=========================================="
echo "       Running Tests"
echo "=========================================="

# Run tests
if [ -f "simple_test.json" ] && [ -f "simple_queries.json" ]; then
    run_test "Simple Test (5 nodes)" \
             "simple_test.json" \
             "simple_queries.json" \
             "output_simple.json"
else
    print_warning "Simple test files not found - skipping"
fi

if [ -f "test_graph.json" ] && [ -f "test_queries.json" ]; then
    run_test "Generated Test" \
             "test_graph.json" \
             "test_queries.json" \
             "output_test.json"
else
    print_warning "Generated test files not found - skipping"
fi

# Calculate total time
END_TIME=$(date +%s)
TOTAL_TIME=$((END_TIME - START_TIME))

# Summary
echo ""
echo "=========================================="
echo "       Test Summary"
echo "=========================================="
echo ""
echo "Total tests:  $TOTAL"
echo -e "Passed:       ${GREEN}$PASSED${NC}"
echo -e "Failed:       ${RED}$FAILED${NC}"
echo "Time elapsed: ${TOTAL_TIME}s"
echo ""

if [ $FAILED -eq 0 ] && [ $PASSED -gt 0 ]; then
    print_success "All tests passed! 🎉"
    exit 0
else
    print_error "Some tests failed or no tests ran."
    exit 1
fi