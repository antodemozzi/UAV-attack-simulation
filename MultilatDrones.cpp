#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include <iostream>
#include <map>
#include <vector>
#include <cmath>

using namespace ns3;// lo usano pure nel tutorial

// Il pacchetto che viaggia via radio
struct MessaggioBeacon {
    uint32_t idCamion; 
    double x;          
    double y;          
    int64_t tempo;     
};

// Struttura interna per salvare i dati ricevuti in memoria
struct DatiAncora {
    double x, y;
    double distanzaCalcolata;
};


std::map<uint32_t, DatiAncora> g_memoriaMisure; 
//fare funzione per risolvere la multilat
Vector RisolviMultilateration() {

}

//qui invio
void InviaPosizione(Ptr<Socket> socket, Ptr<Node> nodo) {
    Vector pos = nodo->GetObject<MobilityModel>()->GetPosition();

    MessaggioBeacon msg;
    msg.idCamion = nodo->GetId(); // Importante: dico chi sono
    msg.x = pos.x;
    msg.y = pos.y;
    msg.tempo = Simulator::Now().GetNanoSeconds();

    Ptr<Packet> p = Create<Packet>((uint8_t*)&msg, sizeof(msg));
    socket->SendTo(p, 0, InetSocketAddress(Ipv4Address::GetBroadcast(), 8080));

    // Loop ogni 0.1 secondi
    Simulator::Schedule(Seconds(0.1), &InviaPosizione, socket, nodo);
}

//ricezione del drone 
void RiceviPosizione(Ptr<Socket> socket) {
    Ptr<Packet> p;
    while ((p = socket->Recv())) {
        MessaggioBeacon msg;
        p->CopyData((uint8_t*)&msg, sizeof(msg));

	std::cout << "-> Rx da Camion " << msg.idCamion << " (pos " << msg.x << "," << msg.y << ")" << "\n";

        
        double secVolo = (Simulator::Now().GetNanoSeconds() - msg.tempo) / 1e9;
        double distTotale = secVolo * 299792458.0;

        
        // Dobbiamo togliere la differenza di altezza per avere la distanza sul piano
        double hDrone = 100.0;
        double hCamion = 0.0; 
        double deltaZ = hDrone - hCamion;
        double distPiana = 0.0;
        
        if (distTotale > deltaZ) {
            distPiana = std::sqrt(distTotale*distTotale - deltaZ*deltaZ);
        }

        
        DatiAncora nuovaAncora;
        nuovaAncora.x = msg.x;
        nuovaAncora.y = msg.y;
        nuovaAncora.distanzaCalcolata = distPiana;

        // Inseriamo o aggiorniamo la misura per questo specifico ID camion
        g_memoriaMisure[msg.idCamion] = nuovaAncora;

        
        // Proviamo a risolvere solo se abbiamo dati da almeno 3 camion diversi , chiamando la funzione risolutrice
        if (g_memoriaMisure.size() >= 3) {
            Vector pos = RisolviMultilateration();
            
            // Se pos non è (0,0,0) (controllo grezzo), stampiamo
            if (pos.x != 0 || pos.y != 0) {
                // Stampiamo su console per vedere se funziona
                // Format: Tempo Simulazione | Posizione Vera Drone | Posizione Calcolata
                std::cout << "[t=" << Simulator::Now().GetSeconds() << "s] "
                          << "Posizione STIMATA: (" << pos.x << ", " << pos.y << ")" 
                          << std::endl;
            }
        }
    }
}


int main(int argc, char *argv[]) {
    // Creazione Nodi
    NodeContainer cCamion, cDrone;
    cCamion.Create(4); // 4 Camion
    cDrone.Create(1);  // 1 Drone

    // Setup Wifi
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211a);
    WifiMacHelper mac; mac.SetType("ns3::AdhocWifiMac");
    YansWifiPhyHelper phy;
    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    phy.SetChannel(channel.Create());

    NetDeviceContainer devCamion = wifi.Install(phy, mac, cCamion);
    NetDeviceContainer devDrone = wifi.Install(phy, mac, cDrone);

    // Setup Mobilità
    MobilityHelper mob;
    
    // Camion in movimento
    mob.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mob.Install(cCamion);
    for(int i=0; i<4; i++) {
        auto m = cCamion.Get(i)->GetObject<ConstantVelocityMobilityModel>();
        
        // TRUCCO: Spostiamo leggermente la X (zig-zag: 0, 10, 0, 10)
        // Così il determinante della matrice non è zero!
        double offsetX = (i % 2 == 0) ? 0.0 : 10.0; 
        
        m->SetPosition(Vector(offsetX, i*30, 0)); // Partono da Y=0, 30, 60...
        m->SetVelocity(Vector(15, 0, 0));

    }

    // Drone fermo (Target)
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mob.Install(cDrone);
    // Posizioniamo il drone
    cDrone.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(50, 50, 20));

    // Setup IP
    InternetStackHelper internet;
    internet.Install(cCamion); internet.Install(cDrone);
    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    ipv4.Assign(devCamion); ipv4.Assign(devDrone);

    // Avvio Applicazioni
    // Camion (Invio)
    for(int i=0; i<4; i++) {
        Ptr<Socket> s = Socket::CreateSocket(cCamion.Get(i), UdpSocketFactory::GetTypeId());
        s->SetAllowBroadcast(true);
        Simulator::Schedule(Seconds(1.0 + i*0.02), &InviaPosizione, s, cCamion.Get(i));
    }

    // Drone (Ricezione)
    Ptr<Socket> sRx = Socket::CreateSocket(cDrone.Get(0), UdpSocketFactory::GetTypeId());
    sRx->Bind(InetSocketAddress(Ipv4Address::GetAny(), 8080));
    sRx->SetRecvCallback(MakeCallback(&RiceviPosizione));

    // Esecuzione simulazioe annamo
    NS_LOG_UNCOND("Inizio simulazione Multilateration...");
    Simulator::Stop(Seconds(5.0));
    Simulator::Run();
    Simulator::Destroy();
    return 0;
}

