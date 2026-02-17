
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include <map>
#include <vector>
#include <cmath>
#include <iostream>

/*
 * DronesTest.cc
 * 
 * Unit test suite for the Multilateration algorithm used in DronesFinal.cc.
 * Verifies that the position estimation works correctly for:
 * 1. Standard non-collinear anchors.
 * 2. Linear anchors + 1 off-axis anchor (Hybrid scenario).
 */

using namespace ns3;

// --- Data Structures (Copied from DronesFinal.cc) ---
struct AnchorData {
    double x, y;
    double calculatedDistance;
};

// --- Algorithm (Copied from DronesFinal.cc) ---
Vector SolveMultilateration(const std::map<uint32_t, AnchorData>& measurementMemory) {
    if (measurementMemory.size() < 3) {
        return Vector(0,0,0);
    }

    std::vector<AnchorData> allAnchors;
    for (auto const& [id, data] : measurementMemory) {
        allAnchors.push_back(data);
    }

    double bestDet = 0.0;
    Vector bestResult(0,0,0);
    bool foundValid = false;

    int n = allAnchors.size();
    if (n < 3) return Vector(0,0,0);

    for (int i = 0; i < n - 2; ++i) {
        for (int j = i + 1; j < n - 1; ++j) {
            for (int k = j + 1; k < n; ++k) {
                AnchorData p0 = allAnchors[i];
                AnchorData p1 = allAnchors[j];
                AnchorData p2 = allAnchors[k];

                double x0 = p0.x, y0 = p0.y, r0 = p0.calculatedDistance;
                double x1 = p1.x, y1 = p1.y, r1 = p1.calculatedDistance;
                double x2 = p2.x, y2 = p2.y, r2 = p2.calculatedDistance;

                double A = 2 * (x0 - x1);
                double B = 2 * (y0 - y1);
                double C = 2 * (x0 - x2);
                double D = 2 * (y0 - y2);

                double E = (r1*r1 - r0*r0) - (x1*x1 - x0*x0) - (y1*y1 - y0*y0);
                double F = (r2*r2 - r0*r0) - (x2*x2 - x0*x0) - (y2*y2 - y0*y0);

                double det = A * D - B * C;
                
                if (std::abs(det) > std::abs(bestDet)) {
                    bestDet = det;
                    double x_est = (E * D - B * F) / det;
                    double y_est = (A * F - E * C) / det;
                    bestResult = Vector(x_est, y_est, 100.0);
                    if (std::abs(det) > 1.0) foundValid = true;
                }
            }
        }
    }

    if (!foundValid) return Vector(0,0,0);
    return bestResult;
}

// --- Test Helper ---
void AssertPosition(Vector estimated, Vector actual, std::string testName) {
    double dist = std::sqrt(std::pow(estimated.x - actual.x, 2) + std::pow(estimated.y - actual.y, 2));
    if (dist < 1.0) {
        std::cout << "[PASS] " << testName << " (Error: " << dist << " m)" << std::endl;
    } else {
        std::cout << "[FAIL] " << testName << " (Error: " << dist << " m)" << std::endl;
        std::cout << "       Expected: (" << actual.x << ", " << actual.y << ")" << std::endl;
        std::cout << "       Got:      (" << estimated.x << ", " << estimated.y << ")" << std::endl;
    }
}

// --- Main Test Loop ---
int main(int argc, char *argv[]) {
    // 1. Standard Test: 3 Anchors in a Right Triangle
    std::map<uint32_t, AnchorData> test1;
    // Target at (50, 50)
    Vector target(50, 50, 0); 
    
    // Anchor 0 at (0,0) -> Dist approx 70.71
    test1[0] = {0, 0, 70.7106}; 
    // Anchor 1 at (100,0) -> Dist approx 70.71
    test1[1] = {100, 0, 70.7106};
    // Anchor 2 at (0,100) -> Dist approx 70.71
    test1[2] = {0, 100, 70.7106};
    
    Vector res1 = SolveMultilateration(test1);
    AssertPosition(res1, target, "Standard Triangle");


    // 2. Hybrid Test: 3 Linear Anchors + 1 Offset Anchor
    // This simulates the new feature: Trucks are linear (bad), but a Drone is offset (good).
    std::map<uint32_t, AnchorData> test2;
    // Target at (50, 50)
    
    // Collinear Trucks at y=0
    test2[0] = {0, 0, 70.7106};    
    test2[1] = {30, 0, 53.8516};   
    test2[2] = {60, 0, 50.9902};   
    
    // Drone Anchor at (50, 100) -> Dist 50.0
    test2[3] = {50, 100, 50.0}; 
    
    Vector res2 = SolveMultilateration(test2);
    AssertPosition(res2, target, "Hybrid Linear + Drone");

    return 0;
}
