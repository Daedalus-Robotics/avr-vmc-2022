def map(x: float | int,
        in_min: float | int, in_max: float | int,
        out_min: float | int, out_max: float | int
        ) -> float | int:
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def get_min(value1: float | int, value2: float | int):
    return value1 if value1 < value2 else value2


def get_max(value1: float | int, value2: float | int):
    return value1 if value1 > value2 else value2


def constrain(val: float | int, min_val: float | int, max_val: float | int) -> float | int:
    return get_min(max_val, get_max(min_val, val))


def deadzone(value: float | int, min_value: float | int) -> float | int:
    number_type = type(value)
    if value <= min_value:
        return number_type(value)
    else:
        return value
