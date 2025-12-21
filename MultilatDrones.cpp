
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
    uint32_t truckId;   // who am i , sender id
    double x;           // where am , x cordinates
    double y;           // where am i , y cordinates
    int64_t timestamp;  
};
// Internal structure to store received data in the drone's memory
struct AnchorData {
    double x, y;
    double calculatedDistance;
};
//we use a map to avoid duplicates , we recive the anchr's name and we overwrite his data
std::map<uint32_t, AnchorData> g_measurementMemory; 


// the matemathical function takes the map of measurements and returns the estimated position (x, y)
// we can improve redability by slitting in other function or in a class
Vector SolveMultilateration() {
    // If we have fewer than 3 anchors, we cannot solve geometrically (Triangulation requires 3)
    if (g_measurementMemory.size() < 3) {
        return Vector(0,0,0); // Return a zero vector to indicate failure
    }

    // Transfer the first 3 points from the map to a vector for convenience
    std::vector<AnchorData> p;
    int count = 0;
    for (auto const& [id, data] : g_measurementMemory) {
        p.push_back(data);
        count++;
        if (count == 3) break; // We only need 3 equations
    }
    // ---------MATHEMATICS: LINEAR LEAST SQUARES (2D)------------
    // We use the method of subtracting equations to remove unknown quadratic terms.
    // This transforms the problem into a linear system: Ax = b
    // Reference Point: Truck 0
    double x0 = p[0].x, y0 = p[0].y, r0 = p[0].calculatedDistance;
    // Truck 1
    double x1 = p[1].x, y1 = p[1].y, r1 = p[1].calculatedDistance;
    // Truck 2
    double x2 = p[2].x, y2 = p[2].y, r2 = p[2].calculatedDistance;
    // Build Matrix A (2x2)
    // The equation form is: 2(x0 - xi) * x + 2(y0 - yi) * y = ...
    double A = 2 * (x0 - x1);
    double B = 2 * (y0 - y1);
    double C = 2 * (x0 - x2);
    double D = 2 * (y0 - y2);
    // Build the vector of known terms (b)
    // Formula: r_i^2 - r_0^2 - x_i^2 + x_0^2 - y_i^2 + y_0^2
    double E = (r1*r1 - r0*r0) - (x1*x1 - x0*x0) - (y1*y1 - y0*y0);
    double F = (r2*r2 - r0*r0) - (x2*x2 - x0*x0) - (y2*y2 - y0*y0);
    // Solve the 2x2 System using Cramer's Rule
    double det = A * D - B * C;

    // If the determinant is zero, the points are poorly aligned (collinear) and cannot be solved
    if (std::abs(det) < 0.001) return Vector(0,0,0);

    double x_estimated = (E * D - B * F) / det;
    double y_estimated = (A * F - E * C) / det;

    return Vector(x_estimated, y_estimated, 100.0); // Z is fixed at 100 as per requirements
}

//send function on the trucks
void SendPosition(Ptr<Socket> socket, Ptr<Node> node) {
    Vector pos = node->GetObject<MobilityModel>()->GetPosition();
    BeaconMessage msg;
    msg.truckId = node->GetId(); 
    msg.x = pos.x;
    msg.y = pos.y;
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
        std::cout << "-> Rx from Truck " << msg.truckId << " (pos " << msg.x << "," << msg.y << ")" << "\n";
        // Distance = (Time Received - Time Sent) * Speed of Light
        double flightTimeSeconds = (Simulator::Now().GetNanoSeconds() - msg.timestamp) / 1e9;
        double totalDistance = flightTimeSeconds * 299792458.0;
        // We must remove the height difference to get the distance on the ground plane
        double droneHeight = 100.0;
        double truckHeight = 0.0; 
        double deltaZ = droneHeight - truckHeight;
        double planarDistance = 0.0;
        
        if (totalDistance > deltaZ) {
            planarDistance = std::sqrt(totalDistance*totalDistance - deltaZ*deltaZ);
        }

       //save data in global memory
        AnchorData newAnchor;
        newAnchor.x = msg.x;
        newAnchor.y = msg.y;
        newAnchor.calculatedDistance = planarDistance;
        // Insert or update the measurement for this specific truck ID
        g_measurementMemory[msg.truckId] = newAnchor;
        // Try to solve only if we have data from at least 3 different trucks
        if (g_measurementMemory.size() >= 3) {
            Vector pos = SolveMultilateration();
            // Basic check: if pos is not (0,0,0), we have a valid result
            if (pos.x != 0 || pos.y != 0) {
                // Print to console
                // Format: Simulation Time | Estimated Position
                std::cout << "[t=" << Simulator::Now().GetSeconds() << "s] "
                          << "ESTIMATED Position: (" << pos.x << ", " << pos.y << ")" 
                          << "\n";
            }
        }
    }
}

int main(int argc, char *argv[]) {
    NodeContainer cTrucks, cDrone;
    cTrucks.Create(4); // 4 Trucks , wicha re the anchor's
    cDrone.Create(1);  // drones created

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
        
        
        // This ensures the geometry is not a perfect line, avoiding singular matrices!
        double offsetX = (i % 2 == 0) ? 0.0 : 10.0; 
        
        m->SetPosition(Vector(offsetX, i*30, 0)); 
        m->SetVelocity(Vector(15, 0, 0));         // Move along X axis at 15 m/s

    }

    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(cDrone);
    
    cDrone.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(50, 50, 20));

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
    Ptr<Socket> sRx = Socket::CreateSocket(cDrone.Get(0), UdpSocketFactory::GetTypeId());
    sRx->Bind(InetSocketAddress(Ipv4Address::GetAny(), 8080)); // Listen on port 8080
    sRx->SetRecvCallback(MakeCallback(&ReceivePosition));

    std::cout << "starting the simulation ... " << "\n";
    
    Simulator::Stop(Seconds(5.0)); // Run for 5 simulation seconds
    Simulator::Run();
    Simulator::Destroy();
    
    return 0;
}
