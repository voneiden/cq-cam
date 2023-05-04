from typing import Any


class Node:
    def __init__(self, tree, obj, parent=None):
        self.tree = tree
        self.obj = obj
        self.parent = parent
        self.tree.nodes.append(self)
        self.locked = False

    def lock(self):
        self.locked = True

    def branch(self, branches):
        try:
            branches = iter(branches)
        except TypeError:
            branches = iter([branches])

        nodes = [Node(self.tree, branch, parent=self) for branch in branches]
        return nodes

    @property
    def parents(self):
        return [self.parent] + self.parent.parents if self.parent else []

    @property
    def traverse(self):
        return [self] + self.parents

    @property
    def traverse_and_unwrap(self):
        traversed = self.traverse
        return [node.obj for node in traversed]

    def __repr__(self):
        return f"Node[{self.obj}]"


class Tree:
    def __init__(self, root):
        self.nodes = []
        self.root = Node(self, root)

    @property
    def parents(self):
        return [node.parent for node in self.nodes if node.parent]

    @property
    def leaves(self):
        parents = self.parents
        return [node for node in self.nodes if node not in parents]

    @property
    def next_unlocked(self):
        leaves = self.leaves
        return next(node for node in leaves if not node.locked)

    @property
    def sequences(self) -> list[list[Any]]:
        leaves = self.leaves
        sequence_candidates = [node.traverse_and_unwrap for node in leaves]
        sequences = []
        used_nodes = []
        while sequence_candidates:
            sequence_candidates.sort(key=lambda x: len(x), reverse=True)
            longest_sequence = sequence_candidates[0]
            sequences.append(longest_sequence)
            used_nodes += longest_sequence
            sequence_candidates = [
                [seq_node for seq_node in seq if seq_node not in used_nodes]
                for seq in sequence_candidates[1:]
            ]
        return sequences
