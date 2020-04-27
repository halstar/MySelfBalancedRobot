def clamp(value, minimum, maximum):

    if value < minimum:
        return minimum
    elif value > maximum:
        return maximum
    else:
        return value
