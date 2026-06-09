/*
 * Space Workload Traffic Generator
 * PhD Research: Chandraboul
 * 
 * Generates synthetic GNC (Guidance, Navigation, Control) traffic patterns
 * to stress-test shared vs dedicated accelerator configurations.
 */

#ifndef __CUSTOM_ACCEL_SPACE_TRAFFIC_GEN_HH__
#define __CUSTOM_ACCEL_SPACE_TRAFFIC_GEN_HH__

#include "base/random.hh"
#include "base/statistics.hh"
#include "mem/port.hh"
#include "params/SpaceTrafficGen.hh"
#include "sim/clocked_object.hh"
#include "sim/eventq.hh"

#include <queue>
#include <vector>
#include <string>

namespace gem5
{

/**
 * SpaceTrafficGen - Generates realistic spacecraft workload traces
 * 
 * Workload Types:
 * - GNC_ATTITUDE: Attitude determination and control (CORDIC-heavy)
 * - GNC_ORBIT: Orbital mechanics calculations (Matrix-heavy)
 * - GNC_NAVIGATION: Navigation filter (Kalman filter, mixed)
 * - PAYLOAD_IMAGE: Image processing pipeline
 * - PAYLOAD_COMPRESS: Data compression
 * - TELEMETRY: Periodic telemetry generation
 * - COMMAND: Ground command processing (bursty)
 */
class SpaceTrafficGen : public ClockedObject
{
  public:
    PARAMS(SpaceTrafficGen);
    SpaceTrafficGen(const Params &p);
    ~SpaceTrafficGen();

    /* Workload types */
    enum WorkloadType {
        GNC_ATTITUDE,       // 100Hz attitude control loop
        GNC_ORBIT,          // 1Hz orbital propagation
        GNC_NAVIGATION,     // 10Hz navigation filter
        PAYLOAD_IMAGE,      // Imaging pipeline (burst)
        PAYLOAD_COMPRESS,   // Compression (sustained)
        TELEMETRY,          // 1Hz telemetry
        COMMAND,            // Sporadic commands
        MIXED_REALTIME,     // Combined real-time tasks
        STRESS_TEST         // Maximum load stress test
    };

    /* Request to accelerator */
    struct TrafficRequest {
        Tick timestamp;
        WorkloadType type;
        int priority;
        int coreId;
        Addr targetAddr;    // MMIO address
        uint64_t data;
        int size;
        bool isRead;
        Tick deadline;      // Real-time deadline
    };

    /* MMIO Port for sending traffic */
    class TrafficPort : public RequestPort
    {
      private:
        SpaceTrafficGen *gen;
        
      public:
        TrafficPort(const std::string &name, SpaceTrafficGen *_gen)
            : RequestPort(name), gen(_gen) {}
        
        bool recvTimingResp(PacketPtr pkt) override {
            return gen->handleResponse(pkt);
        }
        
        void recvReqRetry() override {
            gen->retry();
        }
    };

    Port &getPort(const std::string &name, PortID idx = InvalidPortID) override;

    /* Control interface */
    void startGeneration();
    void stopGeneration();
    void setWorkloadMix(const std::vector<std::pair<WorkloadType, double>> &mix);
    void burstTraffic(WorkloadType type, int count);

  private:
    /* Parameters */
    WorkloadType primaryWorkload;
    double injectionRate;       // Requests per microsecond
    int numCores;
    bool enableDeadlines;
    Cycles baseLatency;
    
    /* MMIO base addresses for accelerators */
    Addr matrixAccelBase;
    Addr cordicAccelBase;
    Addr compressionBase;
    Addr imageCompBase;
    
    /* State */
    bool generating;
    std::queue<TrafficRequest*> pendingRequests;
    std::vector<std::pair<WorkloadType, double>> workloadMix;
    Random random;
    int requestId;
    
    /* Traffic generation */
    void generateTraffic();
    void sendRequest(TrafficRequest *req);
    bool handleResponse(PacketPtr pkt);
    void retry();
    
    /* Workload-specific generators */
    TrafficRequest* generateAttitudeRequest(int coreId);
    TrafficRequest* generateOrbitRequest(int coreId);
    TrafficRequest* generateNavigationRequest(int coreId);
    TrafficRequest* generateImageRequest(int coreId);
    TrafficRequest* generateCompressRequest(int coreId);
    TrafficRequest* generateTelemetryRequest(int coreId);
    TrafficRequest* generateCommandRequest(int coreId);
    
    TrafficPort trafficPort;
    EventFunctionWrapper generateEvent;
    
    /* Sender state for tracking requests in flight */
    class TrafficRequestSenderState : public Packet::SenderState
    {
      public:
        TrafficRequest *req;
        TrafficRequestSenderState(TrafficRequest *r) : req(r) {}
    };
    
    /* Statistics */
    struct SpaceTrafficGenStats : public statistics::Group
    {
        SpaceTrafficGenStats(SpaceTrafficGen *parent);
        
        statistics::Scalar totalRequests;
        statistics::Scalar completedRequests;
        statistics::Scalar droppedRequests;
        statistics::Scalar deadlinesMet;
        statistics::Scalar deadlinesMissed;
        statistics::Histogram responseLatency;
        statistics::Vector perTypeRequests;
        statistics::Vector perTypeLatency;
        statistics::Scalar matrixRequests;
        statistics::Scalar cordicRequests;
        statistics::Scalar compressionRequests;
    } stats;
};

} // namespace gem5

#endif // __CUSTOM_ACCEL_SPACE_TRAFFIC_GEN_HH__

