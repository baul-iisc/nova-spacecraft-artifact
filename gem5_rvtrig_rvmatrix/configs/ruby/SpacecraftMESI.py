# Copyright (c) 2024 Chandraboul - PhD Research
# Spacecraft Ruby Configuration with MESI Coherence and Garnet NoC
#
# This configuration models the interconnect for spacecraft multicore SoC:
# - MESI two-level cache coherence
# - Garnet mesh NoC for detailed timing
# - Accelerator memory controllers as separate nodes

import math

import m5
from m5.defines import buildEnv
from m5.objects import *

from .Ruby import (
    create_directories,
    create_topology,
    send_evicts,
)


class L1Cache(RubyCache):
    """L1 Cache for spacecraft cores - optimized for real-time"""
    pass


class L2Cache(RubyCache):
    """L2 Cache - shared or private per core"""
    pass


def define_options(parser):
    """Add spacecraft-specific Ruby options"""
    parser.add_argument(
        "--spacecraft-l1i-size", type=str, default="16kB",
        help="L1 instruction cache size"
    )
    parser.add_argument(
        "--spacecraft-l1d-size", type=str, default="16kB",
        help="L1 data cache size"
    )
    parser.add_argument(
        "--spacecraft-l2-size", type=str, default="256kB",
        help="L2 cache size per bank"
    )
    parser.add_argument(
        "--spacecraft-num-l2", type=int, default=4,
        help="Number of L2 cache banks"
    )
    parser.add_argument(
        "--spacecraft-mesh-rows", type=int, default=2,
        help="Number of rows in mesh NoC"
    )
    parser.add_argument(
        "--enable-accel-nodes", action="store_true",
        help="Add accelerators as NoC nodes"
    )


def create_system(
    options, full_system, system, dma_ports, bootmem, ruby_system, cpus
):
    """Create spacecraft Ruby memory system with MESI coherence"""
    
    if buildEnv["PROTOCOL"] != "MESI_Two_Level":
        m5.fatal("SpacecraftMESI requires MESI_Two_Level protocol")

    cpu_sequencers = []
    l1_cntrl_nodes = []
    l2_cntrl_nodes = []
    dma_cntrl_nodes = []
    accel_cntrl_nodes = []

    # Get spacecraft-specific options with defaults
    l1i_size = getattr(options, 'spacecraft_l1i_size', '16kB')
    l1d_size = getattr(options, 'spacecraft_l1d_size', '16kB')
    l2_size = getattr(options, 'spacecraft_l2_size', '256kB')
    num_l2 = getattr(options, 'spacecraft_num_l2', options.num_cpus)
    enable_accel = getattr(options, 'enable_accel_nodes', False)

    l2_bits = int(math.log(num_l2, 2)) if num_l2 > 1 else 0
    block_size_bits = int(math.log(options.cacheline_size, 2))

    # Create L1 controllers for each CPU
    for i in range(options.num_cpus):
        l1i_cache = L1Cache(
            size=l1i_size,
            assoc=4,  # 4-way for real-time predictability
            start_index_bit=block_size_bits,
            is_icache=True,
        )
        l1d_cache = L1Cache(
            size=l1d_size,
            assoc=4,
            start_index_bit=block_size_bits,
            is_icache=False,
        )

        prefetcher = RubyPrefetcher()
        clk_domain = cpus[i].clk_domain

        l1_cntrl = L1Cache_Controller(
            version=i,
            L1Icache=l1i_cache,
            L1Dcache=l1d_cache,
            l2_select_num_bits=l2_bits,
            send_evictions=send_evicts(options),
            prefetcher=prefetcher,
            ruby_system=ruby_system,
            clk_domain=clk_domain,
            transitions_per_cycle=options.ports,
            enable_prefetch=False,  # Disabled for real-time
        )

        cpu_seq = RubySequencer(
            version=i,
            dcache=l1d_cache,
            clk_domain=clk_domain,
            ruby_system=ruby_system,
        )

        l1_cntrl.sequencer = cpu_seq
        exec("ruby_system.l1_cntrl%d = l1_cntrl" % i)

        cpu_sequencers.append(cpu_seq)
        l1_cntrl_nodes.append(l1_cntrl)

        # Connect L1 to network
        l1_cntrl.mandatoryQueue = MessageBuffer()
        l1_cntrl.requestFromL1Cache = MessageBuffer()
        l1_cntrl.requestFromL1Cache.out_port = ruby_system.network.in_port
        l1_cntrl.responseFromL1Cache = MessageBuffer()
        l1_cntrl.responseFromL1Cache.out_port = ruby_system.network.in_port
        l1_cntrl.unblockFromL1Cache = MessageBuffer()
        l1_cntrl.unblockFromL1Cache.out_port = ruby_system.network.in_port

        l1_cntrl.optionalQueue = MessageBuffer()

        l1_cntrl.requestToL1Cache = MessageBuffer()
        l1_cntrl.requestToL1Cache.in_port = ruby_system.network.out_port
        l1_cntrl.responseToL1Cache = MessageBuffer()
        l1_cntrl.responseToL1Cache.in_port = ruby_system.network.out_port

    # Create L2 cache banks
    l2_index_start = block_size_bits + l2_bits

    for i in range(num_l2):
        l2_cache = L2Cache(
            size=l2_size,
            assoc=8,
            start_index_bit=l2_index_start,
        )

        l2_cntrl = L2Cache_Controller(
            version=i,
            L2cache=l2_cache,
            transitions_per_cycle=options.ports,
            ruby_system=ruby_system,
        )

        exec("ruby_system.l2_cntrl%d = l2_cntrl" % i)
        l2_cntrl_nodes.append(l2_cntrl)

        # Connect L2 to network
        l2_cntrl.DirRequestFromL2Cache = MessageBuffer()
        l2_cntrl.DirRequestFromL2Cache.out_port = ruby_system.network.in_port
        l2_cntrl.L1RequestFromL2Cache = MessageBuffer()
        l2_cntrl.L1RequestFromL2Cache.out_port = ruby_system.network.in_port
        l2_cntrl.responseFromL2Cache = MessageBuffer()
        l2_cntrl.responseFromL2Cache.out_port = ruby_system.network.in_port

        l2_cntrl.unblockToL2Cache = MessageBuffer()
        l2_cntrl.unblockToL2Cache.in_port = ruby_system.network.out_port
        l2_cntrl.L1RequestToL2Cache = MessageBuffer()
        l2_cntrl.L1RequestToL2Cache.in_port = ruby_system.network.out_port
        l2_cntrl.responseToL2Cache = MessageBuffer()
        l2_cntrl.responseToL2Cache.in_port = ruby_system.network.out_port

    # Memory controller clock domain
    ruby_system.memctrl_clk_domain = DerivedClockDomain(
        clk_domain=ruby_system.clk_domain, clk_divider=3
    )

    # Create directory controllers
    mem_dir_cntrl_nodes, rom_dir_cntrl_node = create_directories(
        options, bootmem, ruby_system, system
    )
    dir_cntrl_nodes = mem_dir_cntrl_nodes[:]
    if rom_dir_cntrl_node is not None:
        dir_cntrl_nodes.append(rom_dir_cntrl_node)

    for dir_cntrl in dir_cntrl_nodes:
        dir_cntrl.requestToDir = MessageBuffer()
        dir_cntrl.requestToDir.in_port = ruby_system.network.out_port
        dir_cntrl.responseToDir = MessageBuffer()
        dir_cntrl.responseToDir.in_port = ruby_system.network.out_port
        dir_cntrl.responseFromDir = MessageBuffer()
        dir_cntrl.responseFromDir.out_port = ruby_system.network.in_port
        dir_cntrl.requestToMemory = MessageBuffer()
        dir_cntrl.responseFromMemory = MessageBuffer()

    # Create DMA controllers
    for i, dma_port in enumerate(dma_ports):
        dma_seq = DMASequencer(
            version=i, ruby_system=ruby_system, in_ports=dma_port
        )

        dma_cntrl = DMA_Controller(
            version=i,
            dma_sequencer=dma_seq,
            transitions_per_cycle=options.ports,
            ruby_system=ruby_system,
        )

        exec("ruby_system.dma_cntrl%d = dma_cntrl" % i)
        dma_cntrl_nodes.append(dma_cntrl)

        dma_cntrl.mandatoryQueue = MessageBuffer()
        dma_cntrl.responseFromDir = MessageBuffer(ordered=True)
        dma_cntrl.responseFromDir.in_port = ruby_system.network.out_port
        dma_cntrl.requestToDir = MessageBuffer()
        dma_cntrl.requestToDir.out_port = ruby_system.network.in_port

    # Combine all controllers
    all_cntrls = (
        l1_cntrl_nodes + l2_cntrl_nodes + dir_cntrl_nodes + 
        dma_cntrl_nodes + accel_cntrl_nodes
    )

    # Full system IO controller
    if full_system:
        io_seq = DMASequencer(version=len(dma_ports), ruby_system=ruby_system)
        ruby_system._io_port = io_seq
        io_controller = DMA_Controller(
            version=len(dma_ports),
            dma_sequencer=io_seq,
            ruby_system=ruby_system,
        )
        ruby_system.io_controller = io_controller

        io_controller.mandatoryQueue = MessageBuffer()
        io_controller.responseFromDir = MessageBuffer(ordered=True)
        io_controller.responseFromDir.in_port = ruby_system.network.out_port
        io_controller.requestToDir = MessageBuffer()
        io_controller.requestToDir.out_port = ruby_system.network.in_port

        all_cntrls = all_cntrls + [io_controller]

    # Configure network
    ruby_system.network.number_of_virtual_networks = 3
    topology = create_topology(all_cntrls, options)
    
    return (cpu_sequencers, mem_dir_cntrl_nodes, topology)


