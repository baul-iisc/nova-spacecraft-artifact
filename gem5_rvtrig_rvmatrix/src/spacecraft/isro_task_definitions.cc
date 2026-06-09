/*
 * NOVA Processor - ISRO Task Definitions Implementation
 * PhD Research: Futuristic Spacecraft Processor
 */

#include "spacecraft/isro_task_definitions.hh"
#include "base/trace.hh"
#include "debug/ISROTasks.hh"

namespace gem5
{

namespace spacecraft
{

ISROTaskDefinitions::ISROTaskDefinitions(const Params &p)
    : SimObject(p)
{
    DPRINTF(ISROTasks, "ISRO Task Definitions initialized\n");
}

SpacecraftTask ISROTaskDefinitions::createGNCTask()
{
    SpacecraftTask gnc;
    gnc.id = static_cast<int>(ISROTaskId::GNC_LOOP);
    gnc.name = "GNC_Loop";
    gnc.criticality = CriticalityLevel::MISSION_CRITICAL;
    
    // 100 Hz control loop
    gnc.period = 10 * TICK_PER_MS;      // 10ms period
    gnc.deadline = 10 * TICK_PER_MS;    // Must complete within period
    gnc.wcet = 5 * TICK_PER_MS;         // 5ms worst-case execution
    
    gnc.isPreemptible = false;          // Cannot be interrupted
    gnc.coreAffinity = 0;               // Prefer core 0
    
    // Requires navigation accelerators
    gnc.accelNeeds = {AccelType::TRIG_ACCEL, AccelType::MAT_ACCEL};
    gnc.memoryKB = 512;
    gnc.powerWatts = 2.0;
    
    // Active during critical phases
    gnc.activePhases = {
        MissionPhase::LAUNCH,
        MissionPhase::ORBIT_INSERTION,
        MissionPhase::LANDING,
        MissionPhase::NORMAL_OPS
    };
    
    return gnc;
}

SpacecraftTask ISROTaskDefinitions::createStarTrackerTask()
{
    SpacecraftTask starTracker;
    starTracker.id = static_cast<int>(ISROTaskId::STAR_TRACKER);
    starTracker.name = "Star_Tracker";
    starTracker.criticality = CriticalityLevel::OPERATIONAL;
    
    // 10 Hz star tracker update
    starTracker.period = 100 * TICK_PER_MS;
    starTracker.deadline = 100 * TICK_PER_MS;
    starTracker.wcet = 50 * TICK_PER_MS;
    
    starTracker.isPreemptible = true;
    starTracker.coreAffinity = -1;  // Any core
    
    // Image processing + attitude computation
    starTracker.accelNeeds = {AccelType::VPU, AccelType::MAT_ACCEL, AccelType::TRIG_ACCEL};
    starTracker.memoryKB = 2048;  // Image buffer
    starTracker.powerWatts = 3.0;
    
    starTracker.activePhases = {
        MissionPhase::NORMAL_OPS,
        MissionPhase::ORBIT_INSERTION
    };
    
    return starTracker;
}

SpacecraftTask ISROTaskDefinitions::createCraterDetectionTask()
{
    SpacecraftTask craterDetect;
    craterDetect.id = static_cast<int>(ISROTaskId::CRATER_DETECTION);
    craterDetect.name = "Crater_Detection";
    craterDetect.criticality = CriticalityLevel::SAFETY_CRITICAL;
    
    // 10 Hz for landing navigation
    craterDetect.period = 100 * TICK_PER_MS;
    craterDetect.deadline = 100 * TICK_PER_MS;
    craterDetect.wcet = 80 * TICK_PER_MS;
    
    craterDetect.isPreemptible = true;
    craterDetect.coreAffinity = -1;
    
    // Full pipeline: Image -> Features -> Classification -> Pose
    craterDetect.accelNeeds = {AccelType::VPU, AccelType::NPU, 
                               AccelType::MAT_ACCEL, AccelType::TRIG_ACCEL};
    craterDetect.memoryKB = 4096;
    craterDetect.powerWatts = 5.0;  // NPU intensive
    
    craterDetect.activePhases = {
        MissionPhase::LANDING
    };
    
    return craterDetect;
}

SpacecraftTask ISROTaskDefinitions::createHazardAvoidanceTask()
{
    SpacecraftTask hazardAvoid;
    hazardAvoid.id = static_cast<int>(ISROTaskId::HAZARD_AVOIDANCE);
    hazardAvoid.name = "Hazard_Avoidance";
    hazardAvoid.criticality = CriticalityLevel::SAFETY_CRITICAL;
    
    // 5 Hz hazard detection
    hazardAvoid.period = 200 * TICK_PER_MS;
    hazardAvoid.deadline = 200 * TICK_PER_MS;
    hazardAvoid.wcet = 150 * TICK_PER_MS;
    
    hazardAvoid.isPreemptible = false;  // Cannot miss hazard detection
    hazardAvoid.coreAffinity = -1;
    
    // Depth estimation + semantic segmentation
    hazardAvoid.accelNeeds = {AccelType::VPU, AccelType::NPU, AccelType::MAT_ACCEL};
    hazardAvoid.memoryKB = 8192;  // Stereo images
    hazardAvoid.powerWatts = 4.0;
    
    hazardAvoid.activePhases = {
        MissionPhase::LANDING
    };
    
    return hazardAvoid;
}

SpacecraftTask ISROTaskDefinitions::createTerrainRelNavTask()
{
    SpacecraftTask trn;
    trn.id = static_cast<int>(ISROTaskId::TERRAIN_REL_NAV);
    trn.name = "Terrain_Relative_Nav";
    trn.criticality = CriticalityLevel::OPERATIONAL;
    
    // 2 Hz terrain matching
    trn.period = 500 * TICK_PER_MS;
    trn.deadline = 500 * TICK_PER_MS;
    trn.wcet = 400 * TICK_PER_MS;
    
    trn.isPreemptible = true;
    trn.coreAffinity = -1;
    
    // Feature matching against stored maps
    trn.accelNeeds = {AccelType::VPU, AccelType::NPU, AccelType::MAT_ACCEL};
    trn.memoryKB = 16384;  // Map database
    trn.powerWatts = 4.5;
    
    trn.activePhases = {
        MissionPhase::LANDING,
        MissionPhase::NORMAL_OPS
    };
    
    return trn;
}

SpacecraftTask ISROTaskDefinitions::createOrbitPropagationTask()
{
    SpacecraftTask orbitProp;
    orbitProp.id = static_cast<int>(ISROTaskId::ORBIT_PROPAGATION);
    orbitProp.name = "Orbit_Propagation";
    orbitProp.criticality = CriticalityLevel::OPERATIONAL;
    
    // 1 Hz orbit prediction
    orbitProp.period = 1000 * TICK_PER_MS;
    orbitProp.deadline = 1000 * TICK_PER_MS;
    orbitProp.wcet = 20 * TICK_PER_MS;
    
    orbitProp.isPreemptible = true;
    orbitProp.coreAffinity = -1;
    
    // Numerical integration of equations of motion
    orbitProp.accelNeeds = {AccelType::TRIG_ACCEL, AccelType::MAT_ACCEL};
    orbitProp.memoryKB = 256;
    orbitProp.powerWatts = 0.5;
    
    orbitProp.activePhases = {
        MissionPhase::NORMAL_OPS,
        MissionPhase::ORBIT_INSERTION
    };
    
    return orbitProp;
}

SpacecraftTask ISROTaskDefinitions::createScienceImageTask()
{
    SpacecraftTask sciImage;
    sciImage.id = static_cast<int>(ISROTaskId::SCIENCE_IMAGE);
    sciImage.name = "Science_Image_Processing";
    sciImage.criticality = CriticalityLevel::SCIENCE;
    
    // 0.1 Hz (every 10 seconds)
    sciImage.period = 10000 * TICK_PER_MS;
    sciImage.deadline = 5000 * TICK_PER_MS;  // 5 second deadline
    sciImage.wcet = 4000 * TICK_PER_MS;
    
    sciImage.isPreemptible = true;
    sciImage.coreAffinity = -1;
    
    // Enhancement, compression, feature extraction
    sciImage.accelNeeds = {AccelType::VPU, AccelType::NPU};
    sciImage.memoryKB = 32768;  // Large images
    sciImage.powerWatts = 3.0;
    
    sciImage.activePhases = {
        MissionPhase::NORMAL_OPS,
        MissionPhase::SURFACE_OPS,
        MissionPhase::SCIENCE_OPS
    };
    
    return sciImage;
}

SpacecraftTask ISROTaskDefinitions::createTelemetryTask()
{
    SpacecraftTask telemetry;
    telemetry.id = static_cast<int>(ISROTaskId::TELEMETRY);
    telemetry.name = "Telemetry";
    telemetry.criticality = CriticalityLevel::HOUSEKEEPING;
    
    // 1 Hz housekeeping
    telemetry.period = 1000 * TICK_PER_MS;
    telemetry.deadline = 1000 * TICK_PER_MS;
    telemetry.wcet = 10 * TICK_PER_MS;
    
    telemetry.isPreemptible = true;
    telemetry.coreAffinity = -1;
    
    // No accelerators needed
    telemetry.accelNeeds = {};
    telemetry.memoryKB = 64;
    telemetry.powerWatts = 0.2;
    
    // Active in all phases
    telemetry.activePhases = {
        MissionPhase::LAUNCH,
        MissionPhase::ORBIT_INSERTION,
        MissionPhase::NORMAL_OPS,
        MissionPhase::LANDING,
        MissionPhase::SURFACE_OPS,
        MissionPhase::SAFE_MODE,
        MissionPhase::ECLIPSE,
        MissionPhase::COMM_WINDOW,
        MissionPhase::SCIENCE_OPS,
        MissionPhase::STANDBY
    };
    
    return telemetry;
}

SpacecraftTask ISROTaskDefinitions::createKalmanFilterTask()
{
    SpacecraftTask kalman;
    kalman.id = static_cast<int>(ISROTaskId::KALMAN_FILTER);
    kalman.name = "Kalman_Filter";
    kalman.criticality = CriticalityLevel::MISSION_CRITICAL;
    
    // 50 Hz sensor fusion
    kalman.period = 20 * TICK_PER_MS;
    kalman.deadline = 20 * TICK_PER_MS;
    kalman.wcet = 8 * TICK_PER_MS;
    
    kalman.isPreemptible = false;
    kalman.coreAffinity = 0;  // Same core as GNC for cache locality
    
    // Matrix operations for state estimation
    kalman.accelNeeds = {AccelType::MAT_ACCEL};
    kalman.memoryKB = 256;
    kalman.powerWatts = 1.0;
    
    kalman.activePhases = {
        MissionPhase::LAUNCH,
        MissionPhase::ORBIT_INSERTION,
        MissionPhase::LANDING,
        MissionPhase::NORMAL_OPS
    };
    
    return kalman;
}

SpacecraftTask ISROTaskDefinitions::createAttitudeControlTask()
{
    SpacecraftTask attitudeCtrl;
    attitudeCtrl.id = static_cast<int>(ISROTaskId::ATTITUDE_CONTROL);
    attitudeCtrl.name = "Attitude_Control";
    attitudeCtrl.criticality = CriticalityLevel::MISSION_CRITICAL;
    
    // 20 Hz attitude control
    attitudeCtrl.period = 50 * TICK_PER_MS;
    attitudeCtrl.deadline = 50 * TICK_PER_MS;
    attitudeCtrl.wcet = 15 * TICK_PER_MS;
    
    attitudeCtrl.isPreemptible = false;
    attitudeCtrl.coreAffinity = 0;
    
    // PID control with rotation matrices
    attitudeCtrl.accelNeeds = {AccelType::MAT_ACCEL, AccelType::TRIG_ACCEL};
    attitudeCtrl.memoryKB = 128;
    attitudeCtrl.powerWatts = 1.5;
    
    attitudeCtrl.activePhases = {
        MissionPhase::LAUNCH,
        MissionPhase::ORBIT_INSERTION,
        MissionPhase::LANDING,
        MissionPhase::NORMAL_OPS
    };
    
    return attitudeCtrl;
}

std::vector<SpacecraftTask> ISROTaskDefinitions::getTasksForPhase(MissionPhase phase)
{
    std::vector<SpacecraftTask> phaseTasks;
    std::vector<SpacecraftTask> allTasks = getAllTasks();
    
    for (const auto &task : allTasks) {
        if (task.activePhases.find(phase) != task.activePhases.end()) {
            phaseTasks.push_back(task);
        }
    }
    
    return phaseTasks;
}

std::vector<SpacecraftTask> ISROTaskDefinitions::getAllTasks()
{
    return {
        createGNCTask(),
        createStarTrackerTask(),
        createCraterDetectionTask(),
        createHazardAvoidanceTask(),
        createTerrainRelNavTask(),
        createOrbitPropagationTask(),
        createScienceImageTask(),
        createTelemetryTask(),
        createKalmanFilterTask(),
        createAttitudeControlTask()
    };
}

SpacecraftTask ISROTaskDefinitions::getTaskById(ISROTaskId id)
{
    switch (id) {
        case ISROTaskId::GNC_LOOP: return createGNCTask();
        case ISROTaskId::STAR_TRACKER: return createStarTrackerTask();
        case ISROTaskId::CRATER_DETECTION: return createCraterDetectionTask();
        case ISROTaskId::HAZARD_AVOIDANCE: return createHazardAvoidanceTask();
        case ISROTaskId::TERRAIN_REL_NAV: return createTerrainRelNavTask();
        case ISROTaskId::ORBIT_PROPAGATION: return createOrbitPropagationTask();
        case ISROTaskId::SCIENCE_IMAGE: return createScienceImageTask();
        case ISROTaskId::TELEMETRY: return createTelemetryTask();
        case ISROTaskId::KALMAN_FILTER: return createKalmanFilterTask();
        case ISROTaskId::ATTITUDE_CONTROL: return createAttitudeControlTask();
        default: return SpacecraftTask();
    }
}

void ISROTaskDefinitions::registerAllTasks(GlobalTaskScheduler *scheduler)
{
    std::vector<SpacecraftTask> allTasks = getAllTasks();
    
    for (const auto &task : allTasks) {
        scheduler->registerTask(task);
    }
    // Note: Static function cannot use DPRINTF (no 'name()' available)
}

void ISROTaskDefinitions::registerTasksForPhase(GlobalTaskScheduler *scheduler, MissionPhase phase)
{
    std::vector<SpacecraftTask> phaseTasks = getTasksForPhase(phase);
    
    for (const auto &task : phaseTasks) {
        scheduler->registerTask(task);
    }
    // Note: Static function cannot use DPRINTF (no 'name()' available)
}

TaskCharacteristics getTaskCharacteristics(ISROTaskId id)
{
    TaskCharacteristics chars;
    chars.id = id;
    
    switch (id) {
        case ISROTaskId::GNC_LOOP:
            chars.name = "GNC_Loop";
            chars.description = "Guidance, Navigation, and Control main loop";
            chars.estimatedInstructions = 50000;
            chars.estimatedMemoryAccesses = 5000;
            chars.computeIntensity = 10.0;
            chars.usesTrigAccel = true;
            chars.usesMatAccel = true;
            chars.usesVPU = false;
            chars.usesNPU = false;
            chars.inputDataSizeBytes = 1024;
            chars.outputDataSizeBytes = 256;
            chars.isHardRealTime = true;
            chars.jitterTolerancePercent = 1.0;
            break;
            
        case ISROTaskId::STAR_TRACKER:
            chars.name = "Star_Tracker";
            chars.description = "Star field recognition for attitude determination";
            chars.estimatedInstructions = 500000;
            chars.estimatedMemoryAccesses = 100000;
            chars.computeIntensity = 5.0;
            chars.usesTrigAccel = true;
            chars.usesMatAccel = true;
            chars.usesVPU = true;
            chars.usesNPU = false;
            chars.inputDataSizeBytes = 640 * 480;  // VGA image
            chars.outputDataSizeBytes = 64;  // Attitude quaternion
            chars.isHardRealTime = false;
            chars.jitterTolerancePercent = 10.0;
            break;
            
        case ISROTaskId::CRATER_DETECTION:
            chars.name = "Crater_Detection";
            chars.description = "Real-time crater detection for lunar/planetary landing";
            chars.estimatedInstructions = 10000000;
            chars.estimatedMemoryAccesses = 2000000;
            chars.computeIntensity = 5.0;
            chars.usesTrigAccel = true;
            chars.usesMatAccel = true;
            chars.usesVPU = true;
            chars.usesNPU = true;
            chars.inputDataSizeBytes = 1024 * 768;
            chars.outputDataSizeBytes = 4096;  // Detected craters
            chars.isHardRealTime = true;
            chars.jitterTolerancePercent = 5.0;
            break;
            
        case ISROTaskId::KALMAN_FILTER:
            chars.name = "Kalman_Filter";
            chars.description = "Extended Kalman Filter for sensor fusion";
            chars.estimatedInstructions = 30000;
            chars.estimatedMemoryAccesses = 3000;
            chars.computeIntensity = 10.0;
            chars.usesTrigAccel = false;
            chars.usesMatAccel = true;
            chars.usesVPU = false;
            chars.usesNPU = false;
            chars.inputDataSizeBytes = 512;
            chars.outputDataSizeBytes = 256;
            chars.isHardRealTime = true;
            chars.jitterTolerancePercent = 2.0;
            break;
            
        default:
            chars.name = "Unknown";
            chars.description = "Unknown task";
            break;
    }
    
    return chars;
}

} // namespace spacecraft
} // namespace gem5

