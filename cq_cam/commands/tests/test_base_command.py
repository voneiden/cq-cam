import unittest

from cadquery import cq

from cq_cam.commands.base_command import CommandSequence
from cq_cam.commands.command import Cut, CircularCW, CircularCCW


def infinite_generator(f):
    while True:
        yield f()


class CommandSequenceTest(unittest.TestCase):
    def setUp(self):
        pass

    def test_reverse_line(self):
        start = cq.Vector(0, 0, 0)
        end = cq.Vector(1, 0, 0)
        initial_cut = Cut(1, None, None)
        reversed_cut = Cut(0, None, None)

        cs = CommandSequence(start, [initial_cut], end)

        self.assertEqual(start, cs.start)
        self.assertEqual(cs.commands[0], initial_cut)
        self.assertEqual(len(cs.commands), 1)
        self.assertEqual(end, cs.end)

        cs.reverse()

        self.assertEqual(start, cs.end)
        self.assertEqual(cs.commands[0], reversed_cut)
        self.assertEqual(len(cs.commands), 1)
        self.assertEqual(end, cs.start)

        cs.reverse()
        self.assertEqual(start, cs.start)
        self.assertEqual(cs.commands[0], initial_cut)
        self.assertEqual(len(cs.commands), 1)
        self.assertEqual(end, cs.end)

    def test_reverse_arc(self):
        start = cq.Vector(0, 0, 0)
        end = cq.Vector(1, 1, 0)
        initial_cut = CircularCW(1, 1, None, 1, (1, 0, 0), None)
        reversed_cut = CircularCCW(0, 0, None, 1, (0, -1, 0), None)

        cs = CommandSequence(start, [initial_cut], end)

        self.assertEqual(start, cs.start)
        self.assertEqual(cs.commands[0], initial_cut)
        self.assertEqual(len(cs.commands), 1)
        self.assertEqual(end, cs.end)

        cs.reverse()

        self.assertEqual(start, cs.end)
        self.assertEqual(cs.commands[0], reversed_cut)
        self.assertEqual(len(cs.commands), 1)
        self.assertEqual(end, cs.start)

        cs.reverse()
        self.assertEqual(start, cs.start)
        self.assertEqual(cs.commands[0], initial_cut)
        self.assertEqual(len(cs.commands), 1)
        self.assertEqual(end, cs.end)
