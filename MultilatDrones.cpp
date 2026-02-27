
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include <iostream>
#include <map>
#include <vector>
#include <cmath>

using namespace ns3;

// The packet payload sent via radio
struct BeaconMessage {
    uint32_t nodeId;    // Renamed from truckId to nodeId (can be truck or drone)
    double x;           // where am , x cordinates
    double y;           // where am i , y cordinates
    double z;           // Added Z coordinate
    int64_t timestamp;  
};
// Internal structure to store received data in the drone's memory
struct AnchorData {
    double x, y;
    double calculatedDistance;
};
//we use a map to avoid duplicates , we recive the anchr's name and we overwrite his data
// Map: DroneID -> (NodeID -> AnchorData)  — keeps only the LATEST measurement per anchor
std::map<uint32_t, std::map<uint32_t, AnchorData>> g_allDronesMemory; 

// Global error tracking
double g_totalError = 0.0;
uint64_t g_errorCount = 0;


// Solve position with exactly 2 anchors using circle intersection
// Returns the intersection point farther from the line between anchors (the "outside" point)
Vector Solve2Anchors(const AnchorData& a1, const AnchorData& a2) {
    double dx = a2.x - a1.x;
    double dy = a2.y - a1.y;
    double d = std::sqrt(dx*dx + dy*dy);
    
    if (d < 1e-6) return Vector(0,0,0); // Anchors at same position
    
    double r1 = a1.calculatedDistance;
    double r2 = a2.calculatedDistance;
    
    // Clamp 'a' so that h^2 doesn't go negative (handles non-intersecting circles)
    double a = (r1*r1 - r2*r2 + d*d) / (2.0 * d);
    double h2 = r1*r1 - a*a;
    double h = (h2 > 0) ? std::sqrt(h2) : 0.0;
    
    // Point on the line between centers at distance 'a' from anchor 1
    double px = a1.x + a * dx / d;
    double py = a1.y + a * dy / d;
    
    // Two intersection points (perpendicular to the line between anchors)
    double ix1 = px + h * dy / d;
    double iy1 = py - h * dx / d;
    double ix2 = px - h * dy / d;
    double iy2 = py + h * dx / d;
    
    // Heuristic: pick the point farther from the anchor midpoint
    // This works because the drone is typically outside the convoy
    double midX = (a1.x + a2.x) / 2.0;
    double midY = (a1.y + a2.y) / 2.0;
    double dist1 = std::sqrt((ix1-midX)*(ix1-midX) + (iy1-midY)*(iy1-midY));
    double dist2 = std::sqrt((ix2-midX)*(ix2-midX) + (iy2-midY)*(iy2-midY));
    
    if (dist1 >= dist2) {
        return Vector(ix1, iy1, 100.0);
    } else {
        return Vector(ix2, iy2, 100.0);
    }
}

// the matemathical function takes the map of measurements and returns the estimated position (x, y)
// we can improve redability by slitting in other function or in a class
Vector SolveMultilateration(const std::map<uint32_t, AnchorData>& measurementMemory) {
    
    // === 2-anchor case: circle intersection ===
    if (measurementMemory.size() == 2) {
        auto it = measurementMemory.begin();
        const AnchorData& a1 = it->second; ++it;
        const AnchorData& a2 = it->second;
        return Solve2Anchors(a1, a2);
    }
    
    if (measurementMemory.size() < 2) {
        return Vector(0,0,0);
    }

    // === 3+ anchor case: standard trilateration ===
    std::vector<AnchorData> allAnchors;
    for (auto const& [id, data] : measurementMemory) {
        allAnchors.push_back(data);
    }

    // Try all combinations of 3 points to find a non-collinear set
    // We prefer a set with a large determinant
    double bestDet = 0.0;
    Vector bestResult(0,0,0);
    bool foundValid = false;

    int n = allAnchors.size();
    for (int i = 0; i < n - 2; ++i) {
        for (int j = i + 1; j < n - 1; ++j) {
            for (int k = j + 1; k < n; ++k) {
                // Get 3 points
                AnchorData p0 = allAnchors[i];
                AnchorData p1 = allAnchors[j];
                AnchorData p2 = allAnchors[k];

                double x0 = p0.x, y0 = p0.y, r0 = p0.calculatedDistance;
                double x1 = p1.x, y1 = p1.y, r1 = p1.calculatedDistance;
                double x2 = p2.x, y2 = p2.y, r2 = p2.calculatedDistance;

                // Linearize: Ax + By = C
                double A = 2 * (x0 - x1);
                double B = 2 * (y0 - y1);
                double C = 2 * (x0 - x2);
                double D = 2 * (y0 - y2);

                double E = (r1*r1 - r0*r0) - (x1*x1 - x0*x0) - (y1*y1 - y0*y0);
                double F = (r2*r2 - r0*r0) - (x2*x2 - x0*x0) - (y2*y2 - y0*y0);

                double det = A * D - B * C;
                
                // Check if this determinant is better (larger abs value means further from collinear)
                if (std::abs(det) > std::abs(bestDet)) {
                    bestDet = det;
                    double x_est = (E * D - B * F) / det;
                    double y_est = (A * F - E * C) / det;
                    bestResult = Vector(x_est, y_est, 100.0);
                    if (std::abs(det) > 0.01) foundValid = true;
                }
            }
        }
    }

    if (!foundValid) {
        // FALLBACK: Least-squares approach for collinear or near-collinear anchors
        int m = allAnchors.size();
        double ATA00=0, ATA01=0, ATA11=0, ATb0=0, ATb1=0;
        double x0_ref = allAnchors[0].x, y0_ref = allAnchors[0].y, r0_ref = allAnchors[0].calculatedDistance;
        int equations = 0;
        for (int i = 1; i < m; ++i) {
            double ai = 2*(x0_ref - allAnchors[i].x);
            double bi = 2*(y0_ref - allAnchors[i].y);
            double ci = (allAnchors[i].calculatedDistance*allAnchors[i].calculatedDistance - r0_ref*r0_ref)
                       -(allAnchors[i].x*allAnchors[i].x - x0_ref*x0_ref)
                       -(allAnchors[i].y*allAnchors[i].y - y0_ref*y0_ref);
            ATA00 += ai*ai; ATA01 += ai*bi; ATA11 += bi*bi;
            ATb0 += ai*ci;  ATb1 += bi*ci;
            equations++;
        }
        double detLS = ATA00*ATA11 - ATA01*ATA01;
        if (equations >= 2 && std::abs(detLS) > 1e-6) {
            double x_est = (ATb0*ATA11 - ATA01*ATb1) / detLS;
            double y_est = (ATA00*ATb1 - ATb0*ATA01) / detLS;
            return Vector(x_est, y_est, 100.0);
        }
        return Vector(0,0,0);
    }
    return bestResult;
}

//send function on the trucks
void SendPosition(Ptr<Socket> socket, Ptr<Node> node) {
    Vector pos = node->GetObject<MobilityModel>()->GetPosition();
    BeaconMessage msg;
    msg.nodeId = node->GetId(); 
    msg.x = pos.x;
    msg.y = pos.y;
    msg.z = pos.z;
    msg.timestamp = Simulator::Now().GetNanoSeconds(); 

    Ptr<Packet> p = Create<Packet>((uint8_t*)&msg, sizeof(msg));
 
    socket->SendTo(p, 0, InetSocketAddress(Ipv4Address::GetBroadcast(), 8080));


    Simulator::Schedule(Seconds(0.1), &SendPosition, socket, node);
}

void ReceivePosition(Ptr<Socket> socket) {
    Ptr<Packet> p;
    while ((p = socket->Recv())) {
        BeaconMessage msg;
        p->CopyData((uint8_t*)&msg, sizeof(msg)); // Deserialize data
        
        uint32_t myId = socket->GetNode()->GetId();
        if (msg.nodeId == myId) continue; // Skip own broadcasted packets, process the rest
        
        /* 
         * NOTE: Time-of-Flight (TOF) ranging in standard ns-3 Wifi models includes MAC access 
         * and queuing delays, leading to huge errors (km range) if not handled with hardware timestamps.
         * For this algorithmic demonstration, we use "Perfect Ranging" derived from ground truth.
         */
         
        Ptr<Node> senderNode = NodeList::GetNode(msg.nodeId);
        double trueDist3D = socket->GetNode()->GetObject<MobilityModel>()->GetDistanceFrom(senderNode->GetObject<MobilityModel>());
        
        // Use actual Z difference
        double myZ = socket->GetNode()->GetObject<MobilityModel>()->GetPosition().z;
        double deltaZ = myZ - msg.z;
        double planarDistance = 0.0;
        
        if (trueDist3D > std::abs(deltaZ)) {
            planarDistance = std::sqrt(trueDist3D*trueDist3D - deltaZ*deltaZ);
        }

        // Save data in global memory — keyed by nodeId so we always keep the LATEST measurement
        // This avoids stale data from when the drone was at a different position
        AnchorData newAnchor;
        newAnchor.x = msg.x;
        newAnchor.y = msg.y;
        newAnchor.calculatedDistance = planarDistance;
        
        g_allDronesMemory[myId][msg.nodeId] = newAnchor;
        
        // Try to solve if we have at least 2 measurements (2-anchor circle intersection supported)
        if (g_allDronesMemory[myId].size() >= 2) {
            Vector pos = SolveMultilateration(g_allDronesMemory[myId]);
            // Basic check: if pos is not (0,0,0), we have a valid result
            if (pos.x != 0 || pos.y != 0) {
                // Get the true position from the simulation ground truth (MobilityModel)
                Ptr<Node> node = socket->GetNode();
                Vector truePos = node->GetObject<MobilityModel>()->GetPosition();

                // Calculate Euclidean distance between Estimated and True position (2D only)
                double errorDistance = std::sqrt(std::pow(pos.x - truePos.x, 2) + std::pow(pos.y - truePos.y, 2));

                // Print both estimated and real positions + error
                std::cout << "[t=" << Simulator::Now().GetSeconds() << "s] Drone " << myId
                          << "  |  ESTIMATED: (" << pos.x << ", " << pos.y << ")"
                          << "  |  REAL: (" << truePos.x << ", " << truePos.y << ")"
                          << "  |  ERROR: " << errorDistance << " m"
                          << "\n";

                // Accumulate error for average calculation
                g_totalError += errorDistance;
                g_errorCount++;

                // SPOOFING DETECTION: Threshold for alarm (e.g. 10 meters)
                if (errorDistance > 10.0) {
                     std::cout << "*** ALARM: Drone " << myId << " POSSIBLE SPOOFING ATTACK! *** (Error: " << errorDistance << "m)" << "\n";
                }
            }
        }
    }
}

int main(int argc, char *argv[]) {
    
    // Command line arguments
    uint32_t nDrones = 2;
    uint32_t nTrucks = 10;      // Number of trucks (anchors)
    double droneSpeedX = 5.0;  // Drone velocity along X axis (m/s)
    double droneSpeedY = 0.0;   // Drone velocity along Y axis (m/s)
    double droneSpeedZ = 0.0;   // Drone velocity along Z axis (m/s)
    double droneStartX = 50.0;  // Drone starting X position (m)
    double droneStartY = 50.0;  // Drone starting Y position (m)
    double droneStartZ = 20.0;  // Drone starting Z position / altitude (m)
    double truckSpeed = 5.0;    // Truck convoy velocity along X axis (m/s)
    std::string anchorMode = "hybrid"; // "trucks", "drones", or "hybrid"
    
    CommandLine cmd;
    cmd.AddValue("nDrones", "Number of drones to simulate", nDrones);
    cmd.AddValue("nTrucks", "Number of trucks (anchors)", nTrucks);
    cmd.AddValue("droneSpeedX", "Drone velocity along X axis (m/s)", droneSpeedX);
    cmd.AddValue("droneSpeedY", "Drone velocity along Y axis (m/s)", droneSpeedY);
    cmd.AddValue("droneSpeedZ", "Drone velocity along Z axis (m/s)", droneSpeedZ);
    cmd.AddValue("droneStartX", "Drone starting X position (m)", droneStartX);
    cmd.AddValue("droneStartY", "Drone starting Y position (m)", droneStartY);
    cmd.AddValue("droneStartZ", "Drone starting altitude (m)", droneStartZ);
    cmd.AddValue("truckSpeed", "Truck convoy velocity along X axis (m/s)", truckSpeed);
    cmd.AddValue("anchorMode", "Anchor mode: trucks, drones, or hybrid", anchorMode);
    cmd.Parse(argc, argv);

    // Validate anchorMode
    if (anchorMode != "trucks" && anchorMode != "drones" && anchorMode != "hybrid") {
        std::cerr << "ERROR: anchorMode must be 'trucks', 'drones', or 'hybrid'. Got: " << anchorMode << std::endl;
        return 1;
    }

    NodeContainer cTrucks, cDrone;
    cTrucks.Create(nTrucks);
    cDrone.Create(nDrones);

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211a); // 5GHz
    // Use constant low data rate for maximum range
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                  "DataMode", StringValue("OfdmRate6Mbps"),
                                  "ControlMode", StringValue("OfdmRate6Mbps"));
    
    WifiMacHelper mac; 
    mac.SetType("ns3::AdhocWifiMac"); // Ad-hoc mode , found on tutorial
    
    YansWifiPhyHelper phy;
    // Increase TX power to extend range (default ~16 dBm is too short for distant nodes)
    phy.Set("TxPowerStart", DoubleValue(30.0));
    phy.Set("TxPowerEnd", DoubleValue(30.0));
    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    phy.SetChannel(channel.Create());

    NetDeviceContainer truckDevices = wifi.Install(phy, mac, cTrucks);
    NetDeviceContainer droneDevices = wifi.Install(phy, mac, cDrone);

    MobilityHelper mobility;
    
   // they moving at costant velocity
    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility.Install(cTrucks);
    
    for(uint32_t i=0; i<nTrucks; i++) {
        auto m = cTrucks.Get(i)->GetObject<ConstantVelocityMobilityModel>();
        
        
        // LINEAR CONVOY (No Zig-Zag)
        double offsetX = 0.0; // All trucks on X=0
        
        m->SetPosition(Vector(offsetX, i*30, 0)); 
        m->SetVelocity(Vector(truckSpeed, 0, 0));   // Truck convoy speed (configurable)

    }

    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility.Install(cDrone);
    
    // Distribute drones and give them a straight-line velocity
    // Each drone starts at a slightly different offset (5m apart) from the base position
    for (uint32_t i = 0; i < nDrones; ++i) {
        auto dm = cDrone.Get(i)->GetObject<ConstantVelocityMobilityModel>();
        dm->SetPosition(Vector(droneStartX + i * 5, droneStartY + i * 5, droneStartZ));
        dm->SetVelocity(Vector(droneSpeedX, droneSpeedY, droneSpeedZ));  // Configurable straight-line velocity
    }

    //ip stack setupp
    InternetStackHelper internet;
    internet.Install(cTrucks); 
    internet.Install(cDrone);
    
    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    ipv4.Assign(truckDevices); 
    ipv4.Assign(droneDevices);
    
    // Trucks send beacons only in "trucks" or "hybrid" mode
    if (anchorMode == "trucks" || anchorMode == "hybrid") {
        for(uint32_t i=0; i<nTrucks; i++) {
            Ptr<Socket> s = Socket::CreateSocket(cTrucks.Get(i), UdpSocketFactory::GetTypeId());
            s->SetAllowBroadcast(true);
            Simulator::Schedule(Seconds(1.0 + i*0.02), &SendPosition, s, cTrucks.Get(i));
        }
    }
    
    // Create a socket for each drone
    for (uint32_t i = 0; i < nDrones; ++i) {
        Ptr<Socket> sRx = Socket::CreateSocket(cDrone.Get(i), UdpSocketFactory::GetTypeId());
        sRx->SetAllowBroadcast(true); // Enable broadcasting for Drones too
        sRx->Bind(InetSocketAddress(Ipv4Address::GetAny(), 8080)); // Listen on port 8080
        sRx->SetRecvCallback(MakeCallback(&ReceivePosition));
        
        // Drones send beacons only in "drones" or "hybrid" mode
        if (anchorMode == "drones" || anchorMode == "hybrid") {
            Simulator::Schedule(Seconds(1.0 + i*0.02 + 0.5), &SendPosition, sRx, cDrone.Get(i));
        }
    }

    std::cout << "starting the simulation ... (anchorMode=" << anchorMode << ")" << "\n";
    
    Simulator::Stop(Seconds(30.0)); // Run for n  simulation seconds
    Simulator::Run();
    Simulator::Destroy();

    // Print average error across all measurements
    if (g_errorCount > 0) {
        double avgError = g_totalError / g_errorCount;
        std::cout << "\n========================================" << "\n";
        std::cout << "SIMULATION RESULTS" << "\n";
        std::cout << "  Anchor mode:        " << anchorMode << "\n";
        std::cout << "  Total measurements: " << g_errorCount << "\n";
        std::cout << "  Average error:      " << avgError << " m" << "\n";
        std::cout << "========================================" << "\n";
    } else {
        std::cout << "\nNo multilateration measurements were recorded." << "\n";
    }
    
    return 0;
}
