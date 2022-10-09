def map(x: float | int,
        in_min: float | int, in_max: float | int,
        out_min: float | int, out_max: float | int
        ) -> float | int:
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def constrain(val: float | int, min_val: float | int, max_val: float | int) -> float | int:
    return min(max_val, max(min_val, val))
