import cadquery as cq

from typing import List


class OCCTClipper:
    outer_boundaries: List[cq.Wire]
    inner_boundaries: List[cq.Wire]
    subjects: List[cq.Wire]

    def __init__(self, outer_boundaries: List[cq.Wire], inner_boundaries: List[cq.Wire], subjects: List[cq.Wire]):
        self.outer_boundaries = outer_boundaries
        self.inner_boundaries = inner_boundaries
        self.subjects = subjects

    def clip(self):
        pass

