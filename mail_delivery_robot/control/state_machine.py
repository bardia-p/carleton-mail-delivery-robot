from enum import Enum
from std_msgs.msg import String

class StateType(Enum):
    OPERATIONAL = "OPERATIONAL"
    NOT_OPERATIONAL = "NOT_OPERATIONAL"
    STOPPED = "STOPPED"
    MOVING = "MOVING"
    HANDLE_COLLISION = "HANDLE_COLLISON"
    IDLE = "IDLE"
    CHARGING = "CHARGING"
    FIND_WALL = "FIND_WALL"
    WALL_FOLLOWING = "WALL_FOLLOWING"
    HANDLE_INTERSECTION = "HANDLE_INTERSECTION"
    DOCKING = "DOCKING"


class State:
    def __init__(self, actionPublisher):
        self.actionPublisher = actionPublisher
        self.stateType = None

    def printState(self):
        return "STATE IS: " + self.stateType.value

    def gotTrip(self):
        return self

    def gotWall(self, data):
        return self

    def lostWall(self):
        return self

    def timeout(self):
        return self

    def needCharge(self):
        return self

    def gotBeacon(self):
        return self

    def gotCollision(self):
        return self

    def gotError(self):
        return self


class Operational(State):
    def __init__(self, actionPublisher):
        super().__init__(actionPublisher)
        self.stateType = StateType.OPERATIONAL

    def timeout(self):
        return Stopped(self.actionPublisher)

    def gotError(self):
        return NotOperationaL(self.actionPublisher)

class NotOperational(State):
    def __init__(self, actionPublisher):
        super().__init__(actionPublisher)
        self.stateType = StateType.NOT_OPERATIONAL

class Stopped(Operational):
    def __init__(self, actionPublisher):
        super().__init__(actionPublisher)
        self.stateType = StateType.STOPPED

    def timeout(self):
        return Idle(self.actionPublisher)


class Moving(Operational):
    def __init__(self, actionPublisher):
        super().__init__(actionPublisher)
        self.stateType = StateType.MOVING

    def timeout(self):
        return FindWall(self.actionPublisher)

class HandleCollision(Operational):
    def __init__(self, actionPublisher):
        super().__init__(actionPublisher)
        self.stateType = StateType.HANDLE_COLLISION

    def timeout(self):
        return FindWall(self.actionPublisher)

class Idle(Stopped):
    def __init__(self, actionPublisher):
        super().__init__(actionPublisher)
        self.stateType = StateType.IDLE

    def gotTrip(self):
        return FindWall(self.actionPublisher)

    def needCharge(self):
        return Charging(self.actionPublisher)

class Charging(Stopped):
    def __init__(self, actionPublisher):
        super().__init__(actionPublisher)
        self.stateType = StateType.CHARGING

    def timeout(self):
        return Idle(self.actionPublisher)

class FindWall(Moving):
    def __init__(self, actionPublisher):
        super().__init__(actionPublisher)
        self.stateType = StateType.FIND_WALL

    def gotWall(self, data):
        action = String()
        action = data
        self.actionPublisher.publish(action)
        return WallFollowing(self.actionPublisher)

    def needCharge(self):
        return NotOperational(self.actionPublisher)

class WallFollowing(Moving):
    def __init__(self, actionPublisher):
        super().__init__(actionPublisher)
        self.stateType = StateType.WALL_FOLLOWING
        self.hasGotBeacon = False
        self.hasLostWall = False

    def lostWall(self):
        self.hasLostWall = True
        if self.hasGotBeacon:
            return HandleIntersection(self.actionPublisher)
        return FindWall(self.actionPublisher)

    def gotWall(self, data):
        self.hasLostWall = False
        action = String()
        action = data
        self.actionPublisher.publish(action)
        return self

    def gotBeacon(self):
        self.hasGotBeacon = True
        if self.hasLostWall:
            return HandleIntersection(self.actionPublisher)
        return self

class HandleIntersection(Moving):
    def __init__(self, actionPublisher):
        super().__init__(actionPublisher)
        self.stateType = StateType.HANDLE_INTERSECTION

    def lostWall(self):
        return FindWall(self.actionPublisher)

    def gotWall(self, data):
        action = String()
        action = data
        self.actionPublisher.publish(action)
        return WallFollowing(self.actionPublisher)

class Docking(Moving):
    def __init__(self, actionPublisher):
        super().__init__(actionPublisher)
        self.stateType = StateType.DOCKING

    def needCharge(self):
        return Idle(self.actionPublisher)

    def timeout(self):
        return Idle(self.actionPublisher)

