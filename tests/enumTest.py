from enum import Enum



class FlightMode(str, Enum):
    HOVER = 'Hover'
    CRUISE = 'Cruise'
    TRANSITION = 'Transition'


if __name__ == "__main__":
    mode = FlightMode.HOVER
    print(mode)
    print(mode.value)
