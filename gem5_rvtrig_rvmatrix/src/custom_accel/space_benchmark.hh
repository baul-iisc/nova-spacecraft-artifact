/*
 * Space Computing Benchmark Suite
 * PhD Research: Chandraboul
 * 
 * Research Question 7: Representative Benchmark Suite for Space Computing
 * - Attitude control algorithms (Kalman, quaternion rotation)
 * - Image processing (star tracking, terrain matching)
 * - Scientific computation (FFT, spectral analysis)
 * - Communication processing (CCSDS encoding, FEC)
 */

#ifndef __CUSTOM_ACCEL_SPACE_BENCHMARK_HH__
#define __CUSTOM_ACCEL_SPACE_BENCHMARK_HH__

#include "base/statistics.hh"
#include "base/trace.hh"
#include "params/SpaceBenchmark.hh"
#include "sim/clocked_object.hh"
#include "sim/eventq.hh"

#include <vector>
#include <map>
#include <functional>
#include <string>

namespace gem5
{

/**
 * SpaceBenchmark - Representative Space Computing Benchmark Suite
 * 
 * Workload Categories:
 * 1. Attitude Determination and Control System (ADCS)
 * 2. Guidance, Navigation, and Control (GNC)
 * 3. Image Processing and Analysis
 * 4. Science Data Processing
 * 5. Telecommunication Processing
 */
class SpaceBenchmark : public ClockedObject
{
  public:
    PARAMS(SpaceBenchmark);
    SpaceBenchmark(const Params &p);
    ~SpaceBenchmark();

    /* Benchmark categories */
    enum BenchmarkCategory {
        CAT_ADCS,           // Attitude control
        CAT_GNC,            // Navigation
        CAT_IMAGE,          // Image processing
        CAT_SCIENCE,        // Science data
        CAT_TELECOM,        // Communication
        NUM_CATEGORIES
    };

    /* Individual benchmarks */
    enum BenchmarkType {
        /* ADCS Benchmarks */
        BENCH_KALMAN_FILTER,        // Extended Kalman filter for attitude
        BENCH_QUATERNION_ROTATION,  // Quaternion-based attitude rotation
        BENCH_GYRO_INTEGRATION,     // Gyroscope data integration
        BENCH_REACTION_WHEEL,       // Reaction wheel control
        
        /* GNC Benchmarks */
        BENCH_ORBIT_PROPAGATION,    // Orbit prediction
        BENCH_TRAJECTORY_CALC,      // Trajectory computation
        BENCH_DOPPLER_NAVIGATION,   // Doppler velocity measurement
        BENCH_TERRAIN_MATCHING,     // Terrain-relative navigation
        
        /* Image Processing */
        BENCH_STAR_TRACKING,        // Star identification
        BENCH_EDGE_DETECTION,       // Edge detection for landing
        BENCH_IMAGE_COMPRESSION,    // CCSDS image compression
        BENCH_FEATURE_EXTRACTION,   // Feature matching
        
        /* Science Data */
        BENCH_FFT,                  // Fast Fourier Transform
        BENCH_SPECTRAL_ANALYSIS,    // Spectral data analysis
        BENCH_DATA_FUSION,          // Sensor fusion
        BENCH_ANOMALY_DETECTION,    // Anomaly detection
        
        /* Telecom */
        BENCH_LDPC_ENCODING,        // LDPC error correction
        BENCH_TURBO_DECODING,       // Turbo code decoding
        BENCH_CCSDS_FRAMING,        // CCSDS packet framing
        BENCH_RANGING,              // Deep space ranging
        
        NUM_BENCHMARKS
    };

    /* Benchmark configuration */
    struct BenchmarkConfig {
        BenchmarkType type;
        std::string name;
        BenchmarkCategory category;
        int dataSize;               // Input data size
        int iterations;             // Number of iterations
        bool useMatrixAccel;        // Needs matrix accelerator
        bool useTrigAccel;          // Needs trig accelerator
        int precisionBits;          // Required precision
        double expectedCycles;      // Expected cycle count
    };

    /* Benchmark result */
    struct BenchmarkResult {
        BenchmarkType type;
        Tick startTime;
        Tick endTime;
        Tick totalCycles;
        double gflops;              // Floating point ops/sec
        double powerUsage;          // Average power
        double efficiency;          // Performance/Watt
        bool passed;                // Correctness check
        double errorMargin;         // Numerical error
    };

    /* Run a specific benchmark */
    BenchmarkResult runBenchmark(BenchmarkType type, 
                                const BenchmarkConfig &config);

    /* Run all benchmarks in a category */
    std::vector<BenchmarkResult> runCategory(BenchmarkCategory category);

    /* Run full benchmark suite */
    std::vector<BenchmarkResult> runFullSuite();

    /* Get benchmark configuration */
    BenchmarkConfig getDefaultConfig(BenchmarkType type);

    /* Register callback for accelerator use */
    void registerMatrixCallback(
        std::function<void(const std::vector<double>&, std::vector<double>&)> cb);
    void registerTrigCallback(
        std::function<void(double, double&)> cb);

    /* Get mission-specific workload mix */
    std::vector<std::pair<BenchmarkType, double>> 
        getMissionWorkloadMix(const std::string &missionType);

  private:
    /* Parameters */
    int defaultIterations;
    int defaultDataSize;
    bool enableValidation;
    bool useHardwareAccel;

    /* State */
    std::vector<BenchmarkResult> allResults;
    std::function<void(const std::vector<double>&, std::vector<double>&)> matrixCallback;
    std::function<void(double, double&)> trigCallback;

    /* Benchmark implementations */
    BenchmarkResult runKalmanFilter(const BenchmarkConfig &config);
    BenchmarkResult runQuaternionRotation(const BenchmarkConfig &config);
    BenchmarkResult runGyroIntegration(const BenchmarkConfig &config);
    BenchmarkResult runOrbitPropagation(const BenchmarkConfig &config);
    BenchmarkResult runStarTracking(const BenchmarkConfig &config);
    BenchmarkResult runFFT(const BenchmarkConfig &config);
    BenchmarkResult runImageCompression(const BenchmarkConfig &config);
    BenchmarkResult runLDPCEncoding(const BenchmarkConfig &config);
    BenchmarkResult runGenericBenchmark(const BenchmarkConfig &config);

    /* Helper functions */
    void initializeData(std::vector<double> &data, int size);
    bool validateResult(const std::vector<double> &result, 
                       const std::vector<double> &expected, 
                       double tolerance);
    double calculateGFLOPS(int64_t ops, Tick cycles);

    /* Default configurations */
    void setupDefaultConfigs();
    std::map<BenchmarkType, BenchmarkConfig> defaultConfigs;

    /* Statistics */
    struct SpaceBenchmarkStats : public statistics::Group
    {
        SpaceBenchmarkStats(SpaceBenchmark *parent);

        statistics::Scalar totalBenchmarks;
        statistics::Scalar passedBenchmarks;
        statistics::Scalar failedBenchmarks;
        statistics::Vector categoryTime;
        statistics::Vector benchmarkTime;
        statistics::Scalar totalGFLOPS;
        statistics::Scalar avgEfficiency;
        statistics::Scalar matrixAccelUsage;
        statistics::Scalar trigAccelUsage;
        statistics::Histogram cycleDistribution;
    } stats;
};

} // namespace gem5

#endif // __CUSTOM_ACCEL_SPACE_BENCHMARK_HH__

