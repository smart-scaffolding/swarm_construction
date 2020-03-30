from queue import PriorityQueue


def merge_paths(divisions):
    values = list(divisions.values())
    values.sort(key=lambda x: x.order)

    item1 = 1
    item2 = 2

    path1 = values[item1].path_to_node
    path2 = values[item2].path_to_node

    # path1 = None
    # path2 = None

    all_nodes = {}
    nodes_to_visit = PriorityQueue()
    # for point in path

    merge = False
    # MERGE = False
    for node in path2:
        # merge = False
        for other_node in path1:
            merge = False
            if node.id == other_node.id:
                # if node in path1:
                # MERGE = True

                # index_in_path = path1.index(node)
                # node_in_path = path1[index_in_path]
                node_in_path = other_node

                if None in node_in_path.direction or None in node.direction:
                    all_nodes[node.id] = (node, None)
                    nodes_to_visit.put(node)
                    merge = True
                    break
                node_in_path.direction = node_in_path.direction.union(node.direction)
                node_in_path.children = node_in_path.children.union(node.children)
                # for direction in node_in_path.num_blocks:
                #     node_in_path.num_blocks[direction] += node.num_blocks[direction]
                node_in_path.num_blocks += node.num_blocks
                merge = True
                break
            # if node in self.need_to_visit_again:
            # MERGE = True
        if not merge:
            #     for other_node in self.need_to_visit_again:
            #         if node.id == other_node.id:
            #             # index_in_path = self.need_to_visit_again.index(node)
            #             # node_in_path = self.need_to_visit_again[index_in_path]
            #             node_in_path = other_node
            #
            #
            #             if None in node_in_path.direction or None in node.direction:
            #                 self.all_nodes[node.id] = (node, None)
            #                 self.nodes_to_visit.put(node)
            #                 merge = True
            #                 continue
            #             node_in_path.direction = node_in_path.direction.union(node.direction)
            #             node_in_path.children = node_in_path.children.union(node.children)
            #             # for direction in node_in_path.num_blocks:
            #             #     node_in_path.num_blocks[direction] += node.num_blocks[direction]
            #             node_in_path.num_blocks += node.num_blocks

            # if not merge:
            # else:
            #     for other_node in need_to_visit_again:
            #         if node.id == other_node.id:
            #             # if node in path1:
            #             # MERGE = True
            #
            #             # index_in_path = path1.index(node)
            #             # node_in_path = path1[index_in_path]
            #             node_in_path = other_node
            #
            #             if None in node_in_path.direction or None in node.direction:
            #                 all_nodes[node.id] = (node, None)
            #                 nodes_to_visit.put(node)
            #                 merge = True
            #                 continue
            #             node_in_path.direction = node_in_path.direction.union(node.direction)
            #             node_in_path.children = node_in_path.children.union(node.children)
            #             # for direction in node_in_path.num_blocks:
            #             #     node_in_path.num_blocks[direction] += node.num_blocks[direction]
            #             node_in_path.num_blocks += node.num_blocks
            #         else:
            all_nodes[node.id] = (node, None)
            nodes_to_visit.put(node)
            merge = False

    # for node in need_to_visit_again:
    #     if node in path1:
    #         # MERGE = True
    #
    #         index_in_path = path1.index(node)
    #         node_in_path = path1[index_in_path]
    #
    #         if None in node_in_path.direction or None in node.direction:
    #             all_nodes[node.id] = (node, None)
    #             nodes_to_visit.put(node)
    #             continue
    #         node_in_path.direction = node_in_path.direction.union(node.direction)
    #         node_in_path.children = node_in_path.children.union(node.children)
    #         # for direction in node_in_path.num_blocks:
    #         #     node_in_path.num_blocks[direction] += node.num_blocks[direction]
    #         node_in_path.num_blocks += node.num_blocks
    #     else:
    #         all_nodes[node.id] = (node, None)
    #         nodes_to_visit.put(node)

    # time.sleep(4) #TODO: Remove Sleep time here
    for node in path1:
        all_nodes[node.id] = (node, None)
        nodes_to_visit.put(node)

    print(nodes_to_visit)


if __name__ == "__main__":
    from components.structure.behaviors.divide_structure import BuildingPlanner
    import numpy as np

    blueprint = np.array([[[1] * 1] * 15] * 15)

    buildingPlanner = BuildingPlanner(blueprint, feeding_location=(0, 0))
    divisions, wavefront_blueprint = buildingPlanner.create_divisions(division_size=5)

    merge_paths(divisions)
