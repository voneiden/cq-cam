from typing import Iterable


def round_array(
    array: Iterable[Iterable[float]], precision=1, inner=tuple, outer=set
) -> Iterable[Iterable[float]]:
    return outer([inner([round(p, precision) for p in point]) for point in array])
