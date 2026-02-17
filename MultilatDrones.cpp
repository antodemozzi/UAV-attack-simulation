
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
// Map: DroneID -> (TruckID -> AnchorData)
std::map<uint32_t, std::map<uint32_t, AnchorData>> g_allDronesMemory; 


// the matemathical function takes the map of measurements and returns the estimated position (x, y)
// we can improve redability by slitting in other function or in a class
Vector SolveMultilateration(const std::map<uint32_t, AnchorData>& measurementMemory) {
    if (measurementMemory.size() < 3) {
        return Vector(0,0,0);
    }

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
                // Threshold 1.0 is arbitrary but safer than 0.001
                if (std::abs(det) > std::abs(bestDet)) {
                    bestDet = det;
                    double x_est = (E * D - B * F) / det;
                    double y_est = (A * F - E * C) / det;
                    bestResult = Vector(x_est, y_est, 100.0); // Z=100 placeholder, not used for distance check
                    if (std::abs(det) > 1.0) foundValid = true;
                }
            }
        }
    }

    if (!foundValid) return Vector(0,0,0);
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
        if (msg.nodeId == myId) return; // Ignore own broadcasted packets

        // Uncomment to see all receptions
        // std::cout << "Node " << myId << " -> Rx from Node " << msg.nodeId << "\n";
        
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

       //save data in global memory
        AnchorData newAnchor;
        newAnchor.x = msg.x;
        newAnchor.y = msg.y;
        newAnchor.calculatedDistance = planarDistance;
        
        // Insert or update the measurement for this specific node ID
        g_allDronesMemory[myId][msg.nodeId] = newAnchor;
        
        // Try to solve only if we have data from at least 3 different nodes
        if (g_allDronesMemory[myId].size() >= 3) {
            Vector pos = SolveMultilateration(g_allDronesMemory[myId]);
            // Basic check: if pos is not (0,0,0), we have a valid result
            if (pos.x != 0 || pos.y != 0) {
                // Print to console
                // Format: Simulation Time | Estimated Position
                std::cout << "[t=" << Simulator::Now().GetSeconds() << "s] Drone " << myId << " "
                          << "ESTIMATED Position: (" << pos.x << ", " << pos.y << ")" 
                          << "\n";
                
                // SPOOFING DETECTION / SAFETY CHECK
                // Get the true position from the simulation ground truth (MobilityModel)
                Ptr<Node> node = socket->GetNode();
                Vector truePos = node->GetObject<MobilityModel>()->GetPosition();

                // Calculate Euclidean distance between Estimated and True position (2D only)
                double errorDistance = std::sqrt(std::pow(pos.x - truePos.x, 2) + std::pow(pos.y - truePos.y, 2));

                // Threshold for alarm (e.g. 10 meters)
                if (errorDistance > 10.0) {
                     std::cout << "*** ALARM: Drone " << myId << " POSSIBLE SPOOFING ATTACK! *** (Error: " << errorDistance << "m)" << "\n";
                }
            }
        }
    }
}

int main(int argc, char *argv[]) {
    
    // Command line argument for number of drones
    uint32_t nDrones = 1;
    CommandLine cmd;
    cmd.AddValue("nDrones", "Number of drones to simulate", nDrones);
    cmd.Parse(argc, argv);

    NodeContainer cTrucks, cDrone;
    cTrucks.Create(4); // 4 Trucks , wicha re the anchor's
    cDrone.Create(nDrones);  // drones created

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211a); // 5GHz 
    
    WifiMacHelper mac; 
    mac.SetType("ns3::AdhocWifiMac"); // Ad-hoc mode , found on tutorial
    
    YansWifiPhyHelper phy;
    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    phy.SetChannel(channel.Create());

    NetDeviceContainer truckDevices = wifi.Install(phy, mac, cTrucks);
    NetDeviceContainer droneDevices = wifi.Install(phy, mac, cDrone);

    MobilityHelper mobility;
    
   // they moving at costant velocity
    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility.Install(cTrucks);
    
    for(int i=0; i<4; i++) {
        auto m = cTrucks.Get(i)->GetObject<ConstantVelocityMobilityModel>();
        
        
        // LINEAR CONVOY (No Zig-Zag)
        double offsetX = 0.0; // All trucks on X=0
        
        m->SetPosition(Vector(offsetX, i*30, 0)); 
        m->SetVelocity(Vector(15, 0, 0));         // Move along X axis at 15 m/s

    }

    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(cDrone);
    
    // Distribute drones to avoid exact overlap and show independence
    // Position them around 50, 50, 20 with some offset based on ID
    for (uint32_t i = 0; i < nDrones; ++i) {
        cDrone.Get(i)->GetObject<MobilityModel>()->SetPosition(Vector(50 + i * 5, 50 + i * 5, 20));
    }

    //ip stack setupp
    InternetStackHelper internet;
    internet.Install(cTrucks); 
    internet.Install(cDrone);
    
    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    ipv4.Assign(truckDevices); 
    ipv4.Assign(droneDevices);
    
    for(int i=0; i<4; i++) {
        Ptr<Socket> s = Socket::CreateSocket(cTrucks.Get(i), UdpSocketFactory::GetTypeId());
        s->SetAllowBroadcast(true);
        Simulator::Schedule(Seconds(1.0 + i*0.02), &SendPosition, s, cTrucks.Get(i));
    }
    
    // Create a socket for each drone
    for (uint32_t i = 0; i < nDrones; ++i) {
        Ptr<Socket> sRx = Socket::CreateSocket(cDrone.Get(i), UdpSocketFactory::GetTypeId());
        sRx->SetAllowBroadcast(true); // Enable broadcasting for Drones too
        sRx->Bind(InetSocketAddress(Ipv4Address::GetAny(), 8080)); // Listen on port 8080
        sRx->SetRecvCallback(MakeCallback(&ReceivePosition));
        
        // Drones now act as anchors too!
        Simulator::Schedule(Seconds(1.0 + i*0.02 + 0.5), &SendPosition, sRx, cDrone.Get(i));
    }

    std::cout << "starting the simulation ... " << "\n";
    
    Simulator::Stop(Seconds(5.0)); // Run for 5 simulation seconds
    Simulator::Run();
    Simulator::Destroy();
    
    return 0;
}
