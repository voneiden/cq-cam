import pytest

from cq_cam.utils.tree import Tree


def test_tree():
    tree = Tree("root")
    l1, m1, r1 = tree.root.branch(["L1", "M1", "R1"])
    l2 = l1.branch(["L2"])[0]
    l3a, l3b = l2.branch(["L3A", "L3B"])
    r1a, r1b, r1c = r1.branch(["R1A", "R1B", "R1C"])
    r2a, r2b = r1b.branch(["R2A", "R2B"])
    r3 = r2b.branch(["R3"])[0]

    assert tree.next_unlocked == m1
    m1.lock()
    assert tree.next_unlocked == l3a
    assert tree.next_unlocked == l3a
    l3a.lock()
    assert tree.next_unlocked == l3b
    l3b.lock()
    assert tree.next_unlocked == r1a
    r1a.lock()
    assert tree.next_unlocked == r1c
    r1c.lock()
    assert tree.next_unlocked == r2a
    r2a.lock()
    assert tree.next_unlocked == r3
    r3.lock()
    with pytest.raises(StopIteration):
        _ = tree.next_unlocked

    sequences = tree.sequences
    assert [
        ["R3", "R2B", "R1B", "R1", "root"],
        ["L3A", "L2", "L1"],
        ["L3B"],
        ["R2A"],
        ["R1A"],
        ["R1C"],
        ["M1"],
    ] == sequences
