# Energy-Proportional Manager SimObject
# PhD Research: Chandraboul

from m5.params import *
from m5.objects.ClockedObject import ClockedObject

class EnergyProportionalManager(ClockedObject):
    """
    Energy-Proportional Accelerator Manager
    
    Manages power consumption across accelerators with:
    - Dynamic Voltage-Frequency Scaling (DVFS)
    - Dark silicon management
    - Solar/battery awareness
    - Thermal management
    
    Research Question 5: Performance-per-watt characterization across orbital conditions
    """
    type = 'EnergyProportionalManager'
    cxx_header = "custom_accel/energy_proportional.hh"
    cxx_class = 'gem5::EnergyProportionalManager'

    # Total power budget (Watts)
    total_power_budget = Param.Float(15.0, "Total power budget in Watts")
    
    # Base system power consumption
    base_power_consumption = Param.Float(2.0, 
        "Base system power consumption in Watts")
    
    # Solar panel capacity (Watts)
    solar_panel_capacity = Param.Float(20.0, 
        "Maximum solar panel output in Watts")
    
    # Battery capacity (Watt-hours)
    battery_capacity = Param.Float(100.0, "Battery capacity in Watt-hours")
    
    # Thermal limit (degrees C)
    thermal_limit = Param.Float(85.0, "Maximum junction temperature")
    
    # Enable DVFS
    enable_dvfs = Param.Bool(True, "Enable dynamic voltage-frequency scaling")
    
    # Enable power gating
    enable_power_gating = Param.Bool(True, "Enable accelerator power gating")
    
    # DVFS transition time
    dvfs_transition_cycles = Param.Cycles(100, "Cycles for DVFS transition")

