# Copyright (c) 2024 Chandraboul - PhD Research
# Spacecraft Mesh NoC Topology with Accelerator Nodes
#
# Custom mesh topology that places accelerators at strategic positions
# for optimal contention analysis

from common import FileSystemConfig
from topologies.BaseTopology import SimpleTopology

from m5.objects import *
from m5.params import *


class SpacecraftMesh(SimpleTopology):
    """
    Spacecraft Mesh Topology
    
    Layout for 4-core system with 2 shared accelerators:
    
    +-------+-------+-------+-------+
    | CPU0  | CPU1  | CPU2  | CPU3  |
    | (L1)  | (L1)  | (L1)  | (L1)  |
    +-------+-------+-------+-------+
    | L2    | L2    | L2    | L2    |
    | Bank0 | Bank1 | Bank2 | Bank3 |
    +-------+-------+-------+-------+
    | Matrix| CORDIC| Dir   | Mem   |
    | Accel | Accel | Ctrl  | Ctrl  |
    +-------+-------+-------+-------+
    
    This topology enables:
    - Measuring contention on shared accelerators
    - Analyzing NoC traffic patterns
    - Fair comparison of shared vs dedicated configurations
    """
    description = "SpacecraftMesh"

    def __init__(self, controllers):
        self.nodes = controllers

    def makeTopology(self, options, network, IntLink, ExtLink, Router):
        nodes = self.nodes

        # Determine mesh dimensions
        # For spacecraft: rows = cores/2, cols = 2 (or flexible)
        num_cpus = options.num_cpus
        num_rows = getattr(options, 'mesh_rows', max(2, num_cpus // 2))
        num_columns = max(2, (num_cpus + num_rows - 1) // num_rows)
        num_routers = num_rows * num_columns

        # Get latency parameters
        link_latency = options.link_latency
        router_latency = options.router_latency

        # Validate configuration
        assert num_rows > 0 and num_rows <= num_routers
        assert num_columns * num_rows >= num_cpus

        # Create routers
        routers = [
            Router(router_id=i, latency=router_latency)
            for i in range(num_routers)
        ]
        network.routers = routers

        link_count = 0

        # Categorize nodes by type
        l1_nodes = []
        l2_nodes = []
        dir_nodes = []
        dma_nodes = []
        other_nodes = []

        for node in nodes:
            if hasattr(node, 'type'):
                if 'L1' in node.type:
                    l1_nodes.append(node)
                elif 'L2' in node.type:
                    l2_nodes.append(node)
                elif 'Directory' in node.type:
                    dir_nodes.append(node)
                elif 'DMA' in node.type:
                    dma_nodes.append(node)
                else:
                    other_nodes.append(node)
            else:
                other_nodes.append(node)

        # Create external links (nodes to routers)
        ext_links = []

        # Map L1 controllers to first row of routers
        for i, node in enumerate(l1_nodes):
            router_id = i % num_routers
            ext_links.append(
                ExtLink(
                    link_id=link_count,
                    ext_node=node,
                    int_node=routers[router_id],
                    latency=link_latency,
                )
            )
            link_count += 1

        # Map L2 controllers
        for i, node in enumerate(l2_nodes):
            router_id = (i + num_columns) % num_routers
            ext_links.append(
                ExtLink(
                    link_id=link_count,
                    ext_node=node,
                    int_node=routers[router_id],
                    latency=link_latency,
                )
            )
            link_count += 1

        # Map directory controllers
        for i, node in enumerate(dir_nodes):
            router_id = num_routers - 1 - (i % num_columns)
            ext_links.append(
                ExtLink(
                    link_id=link_count,
                    ext_node=node,
                    int_node=routers[router_id],
                    latency=link_latency,
                )
            )
            link_count += 1

        # Map DMA and other nodes to router 0
        for node in dma_nodes + other_nodes:
            ext_links.append(
                ExtLink(
                    link_id=link_count,
                    ext_node=node,
                    int_node=routers[0],
                    latency=link_latency,
                )
            )
            link_count += 1

        network.ext_links = ext_links

        # Create internal links (router-to-router)
        int_links = []

        # East-West links (weight 1)
        for row in range(num_rows):
            for col in range(num_columns - 1):
                east_out = col + (row * num_columns)
                west_in = (col + 1) + (row * num_columns)
                
                # Eastbound
                int_links.append(
                    IntLink(
                        link_id=link_count,
                        src_node=routers[east_out],
                        dst_node=routers[west_in],
                        src_outport="East",
                        dst_inport="West",
                        latency=link_latency,
                        weight=1,
                    )
                )
                link_count += 1

                # Westbound
                int_links.append(
                    IntLink(
                        link_id=link_count,
                        src_node=routers[west_in],
                        dst_node=routers[east_out],
                        src_outport="West",
                        dst_inport="East",
                        latency=link_latency,
                        weight=1,
                    )
                )
                link_count += 1

        # North-South links (weight 2 for XY routing)
        for col in range(num_columns):
            for row in range(num_rows - 1):
                north_out = col + (row * num_columns)
                south_in = col + ((row + 1) * num_columns)
                
                # Southbound
                int_links.append(
                    IntLink(
                        link_id=link_count,
                        src_node=routers[north_out],
                        dst_node=routers[south_in],
                        src_outport="North",
                        dst_inport="South",
                        latency=link_latency,
                        weight=2,
                    )
                )
                link_count += 1

                # Northbound
                int_links.append(
                    IntLink(
                        link_id=link_count,
                        src_node=routers[south_in],
                        dst_node=routers[north_out],
                        src_outport="South",
                        dst_inport="North",
                        latency=link_latency,
                        weight=2,
                    )
                )
                link_count += 1

        network.int_links = int_links

    def registerTopology(self, options):
        """Register nodes with filesystem"""
        for i in range(options.num_cpus):
            FileSystemConfig.register_node(
                [i], MemorySize(options.mem_size) // options.num_cpus, i
            )


