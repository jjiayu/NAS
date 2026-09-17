// Directed tests for core/reachability — loads real assets from the
// talosReachability package created in phase 1, to validate the query
// interface against actual data rather than only synthetic inputs.
//
// Deliberately only tests the Forward direction: the mapping from Talos's
// antecedent filenames to (moving_effector, support_effector) is genuinely
// ambiguous in the old code (see docs/paper-deltas.md) and this test isn't
// the place to guess at it.

#include "nas/core/reachability.hpp"

#include <iostream>
#include <stdexcept>
#include <string>

using namespace nas;

namespace {

int g_failures = 0;

void check(bool cond, const std::string& what) {
    if (!cond) {
        std::cerr << "FAIL: " << what << "\n";
        ++g_failures;
    } else {
        std::cout << "ok: " << what << "\n";
    }
}

} // namespace

#ifndef TALOS_REACHABILITY_DATA_DIR
#error "TALOS_REACHABILITY_DATA_DIR must be defined by CMake"
#endif

int main() {
    const std::string data_dir = TALOS_REACHABILITY_DATA_DIR;

    std::vector<ReachabilityEntry> entries = {
        {data_dir + "/RF_constraints_in_LF_quasi_flat_REDUCED.obj", "RF", "LF", ReachabilityDirection::Forward},
        {data_dir + "/LF_constraints_in_RF_quasi_flat_REDUCED.obj", "LF", "RF", ReachabilityDirection::Forward},
    };

    ReachabilityModel model = ReachabilityModel::load(entries);
    check(model.size() == 2, "loaded exactly the 2 forward entries requested");

    check(model.has("RF", "LF", ReachabilityDirection::Forward), "has() finds the loaded RF-in-LF forward entry");
    check(model.has("LF", "RF", ReachabilityDirection::Forward), "has() finds the loaded LF-in-RF forward entry");

    check(!model.has("RF", "LF", ReachabilityDirection::Antecedent),
          "has() correctly reports the (unrequested) antecedent direction as absent");
    check(!model.has("RF", "RR", ReachabilityDirection::Forward),
          "has() correctly reports an unknown effector pair as absent");

    const Polyhedron& rf_in_lf = model.query("RF", "LF", ReachabilityDirection::Forward);
    check(std::distance(rf_in_lf.vertices_begin(), rf_in_lf.vertices_end()) > 0,
          "query() returns a non-empty polytope for a loaded entry");

    bool threw = false;
    try {
        model.query("RF", "LF", ReachabilityDirection::Antecedent);
    } catch (const std::out_of_range&) {
        threw = true;
    }
    check(threw, "query() throws std::out_of_range for a key that was never loaded");

    if (g_failures > 0) {
        std::cerr << g_failures << " test(s) FAILED\n";
        return 1;
    }
    std::cout << "All core/reachability tests passed\n";
    return 0;
}
