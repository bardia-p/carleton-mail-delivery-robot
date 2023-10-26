from enum import Enum
from std_msgs.msg import String

class StateType(Enum):
    OPERATIONAL = "OPERATIONAL"
    NOT_OPERATIONAL = "NOT_OPERATIONAL"
    FIND_WALL = "FIND_WALL"
    WALL_FOLLOWING = "WALL_FOLLOWING"
    DOCKING = "DOCKING"
    PASS_INTERSECTION = "PASS_INTERSECTION"
    TURN_RIGHT = "TURN_RIGHT"
    TURN_LEFT = "TURN_LEFT"
    HANDLE_COLLISION = "HANDLE_COLLISION"

class Direction(Enum):
    NONE = "NONE"
    RIGHT = "RIGHT"
    LEFT = "LEFT"
    PASS = "PASS"
    DOCK = "DOCK"

class State:
    def __init__(self, actionPublisher):
        self.actionPublisher = actionPublisher
        self.stateType = None

    def printState(self):
        return "STATE IS: " + self.stateType.value

    def gotNavLeft(self):
        return self

    def gotNavRight(self):
        return self
    
    def gotNavPass(self):
        return self
    
    def gotNavDock(self):
        return self
    
    def gotWall(self, data):
        return self

    def lostWall(self):
        return self

    def gotBump(self):
        return self

    def gotTrip(self):
        return self
    
    def gotError(self):
        return self


class Operational(State):
    def __init__(self, actionPublisher):
        super().__init__(actionPublisher)
        self.stateType = StateType.OPERATIONAL

    def gotError(self):
        return NotOperational(self.actionPublisher)

class NotOperational(State):
    def __init__(self, actionPublisher):
        super().__init__(actionPublisher)
        self.stateType = StateType.NOT_OPERATIONAL

class FindWall(Operational):
    def __init__(self, actionPublisher):
        super().__init__(actionPublisher)
        self.stateType = StateType.FIND_WALL
        self.actionPublisher.publish(generateAction("R_TURN"))

    def lostWall(self):
        self.actionPublisher.publish(generateAction("R_TURN"))
        return self
    
    def gotWall(self, data):
        return WallFollowing(self.actionPublisher, Direction.NONE, data)

    def gotNavLeft(self):
        return TurnLeft(self.actionPublisher)

    def gotNavRight(self):
        return TurnRight(self.actionPublisher)
    
    def gotNavPass(self):
        return PassIntersection(self.actionPublisher)
    
    def gotNavDock(self):
        return Docking(self.actionPublisher)
    
class WallFollowing(Operational):
    def __init__(self, actionPublisher, nextDir, lidarData = ""):
        super().__init__(actionPublisher)
        self.stateType = StateType.WALL_FOLLOWING
        self.nextDir = nextDir
        self.lidarData = lidarData

    def gotWall(self, data):
        self.actionPublisher.publish(generateAction("WALL_FOLLOW", data))
        return self

    def lostWall(self):
        if (self.nextDir == Direction.LEFT):
            return TurnLeft(self.actionPublisher)
        elif (self.nextDir == Direction.RIGHT):
            return TurnRight(self.actionPublisher)
        elif (self.nextDir == Direction.PASS):
            return PassIntersection(self.actionPublisher)
        elif (self.nextDir == Direction.DOCK):
            return Docking(self.actionPublisher)
        else:
            return FindWall(self.actionPublisher)

    def gotNavLeft(self):
        self.nextDir = Direction.LEFT
        return self

    def gotNavRight(self):
        self.nextDir = Direction.RIGHT
        return self
    
    def gotNavPass(self):
        self.nextDir = Direction.PASS
        return self

    def gotNavDock(self):
        self.nextDir = Direction.DOCK
        return self

class TurnLeft(Operational):
    def __init__(self, actionPublisher):
        super().__init__(actionPublisher)
        self.stateType = StateType.TURN_LEFT
        self.actionPublisher.publish(generateAction("L_TURN"))

    def lostWall(self):
        self.actionPublisher.publish(generateAction("L_TURN"))
        return self
    
    def gotWall(self, data):
        return WallFollowing(self.actionPublisher, Direction.NONE, data)
    
    def gotBump(self):
        return HandleCollision(self.actionPublisher, Direction.LEFT)

class TurnRight(Operational):
    def __init__(self, actionPublisher):
        super().__init__(actionPublisher)
        self.stateType = StateType.TURN_RIGHT
        self.actionPublisher.publish(generateAction("R_TURN"))

    def lostWall(self):
        self.actionPublisher.publish(generateAction("R_TURN"))
        return self
    
    def gotWall(self, data):
        return WallFollowing(self.actionPublisher, Direction.NONE, data)
    
    def gotBump(self):
        return HandleCollision(self.actionPublisher, Direction.RIGHT)
    
class PassIntersection(Operational):
    def __init__(self, actionPublisher):
        super().__init__(actionPublisher)
        self.stateType = StateType.PASS_INTERSECTION
        self.actionPublisher.publish(generateAction("FORWARD"))

    def lostWall(self):
        self.actionPublisher.publish(generateAction("FORWARD"))
        return self
    
    def gotWall(self, data):
        return WallFollowing(self.actionPublisher, Direction.NONE, data)
    
    def gotBump(self):
        return HandleCollision(self.actionPublisher, Direction.PASS)
    
class Docking(Operational):
    def __init__(self, actionPublisher):
        super().__init__(actionPublisher)
        self.stateType = StateType.DOCKING
        self.actionPublisher.publish(generateAction("DOCK"))

    def gotTrip(self, data):
        self.actionPublisher.publish(generateAction("UNDOCK"))
        return FindWall(self.actionPublisher)

class HandleCollision(Operational):
    def __init__(self, actionPublisher, nextDir):
        super().__init__(actionPublisher)
        self.stateType = StateType.HANDLE_COLLISION
        self.nextDir = nextDir
        self.actionPublisher.publish(generateAction("L_TURN"))

    def gotWall(self, data):
        return WallFollowing(self.actionPublisher, self.nextDir, data)

    def lostWall(self):
        if (self.nextDir == Direction.LEFT):
            return TurnLeft(self.actionPublisher)
        elif (self.nextDir == Direction.RIGHT):
            return TurnRight(self.actionPublisher)
        elif (self.nextDir == Direction.PASS):
            return PassIntersection(self.actionPublisher)
        elif (self.nextDir == Direction.DOCK):
            return Docking(self.actionPublisher)
        else:
            return FindWall(self.actionPublisher)

    def gotNavLeft(self):
        self.nextDir = Direction.LEFT
        return self

    def gotNavRight(self):
        self.nextDir = Direction.RIGHT
        return self
    
    def gotNavPass(self):
        self.nextDir = Direction.PASS
        return self

    def gotNavDock(self):
        self.nextDir = Direction.DOCK
        return self
    
def generateAction(command, data = ""):
    action = String()
    action.data = command
    if data != "":
        action.data += ":" + data
    return action


