from enum import Enum
from std_msgs.msg import String

from navigation.captain import Nav_Event

class StateType(Enum):
    OPERATIONAL = "OPERATIONAL"
    NOT_OPERATIONAL = "NOT_OPERATIONAL"
    DOCKED = "DOCKED"
    NO_DEST = "NO_DEST"
    SHOULD_TURN_LEFT = "SHOULD_TURN_LEFT"
    SHOULD_TURN_RIGHT = "SHOULD_TURN_RIGHT"
    SHOULD_PASS = "SHOULD_PASS"
    SHOULD_DOCK = "SHOULD_DOCK"
    HANDLE_INTERSECTION = "HANDLE_INTERSECTION"
    COLLISION_NO_DEST = "COLLISION_NO_DEST"
    COLLISION_TURN_LEFT = "COLLISION_TURN_LEFT"
    COLLISION_TURN_RIGHT = "COLLISION_TURN_RIGHT"
    COLLISION_PASS = "COLLISION_PASS"
    COLLISION_DOCK = "COLLISION_DOCK"
    COLLISION_INTERSECTION = "COLLISION_INTERSECTION"

class Action(Enum):
    L_TURN = "L_TURN"
    R_TURN = "R_TURN"
    FORWARD = "FORWARD"
    DOCK = "DOCK"
    UNDOCK = "UNDOCK"
    WALL_FOLLOW = "WALL_FOLLOW"

class State:
    def __init__(self, actionPublisher):
        self.actionPublisher = actionPublisher
        self.stateType = None
        self.wall = "-1,-1"
        self.longAction = None
        self.longActionCount = 0
        self.longActionLimit = 0
        self.postLongActionState = self

    def printState(self):
        '''
        Displays the state.
        '''
        return "STATE IS: " + self.stateType.value

    # All possible transitions.
    def no_bumper_none_no_wall(self):
        return self

    def no_bumper_none_wall(self):
        return self
    
    def no_bumper_left_no_wall(self):
        return self

    def no_bumper_left_wall(self):
        return self
    
    def no_bumper_right_no_wall(self):
        return self

    def no_bumper_right_wall(self):
        return self
    
    def no_bumper_pass_no_wall(self):
        return self

    def no_bumper_pass_wall(self):
        return self

    def no_bumper_dock_no_wall(self):
        return self

    def no_bumper_dock_wall(self):
        return self
    
    def bumper_none(self):
        return self

    def bumper_left(self):
        return self
    
    def bumper_right(self):
        return self

    def bumper_pass(self):
        return self
    
    def bumper_dock(self):
        return self
    
    def error(self):
        return self

    def handleUpdate(self, bumper, nav, wall):
        '''
        Locates the appropriate state transition based on the update.

        @param bumper: The state of the bumper sensor.
        @param nav: The navigation event.
        @param wall: The wall status.
        '''
        if self.longAction != None:
            if self.longActionCount < self.longActionLimit:
                self.actionPublisher.publish(self.longAction)
                self.longActionCount += 1
                return self
            else:
                self.longActionCount = 0
                self.longActionLimit = 0
                self.longAction = None
                return self.postLongActionState

        if wall == "":
            return self

        self.wall = wall
        if bumper:
            if nav == Nav_Event.NAV_LEFT.value:
                return self.bumper_left()
            elif nav == Nav_Event.NAV_RIGHT.value:
                return self.bumper_right()
            elif nav == Nav_Event.NAV_PASS.value:
                return self.bumper_pass()
            elif nav == Nav_Event.NAV_DOCK.value:
                return self.bumper_dock()
            else:
                return self.bumper_none()
        if nav == Nav_Event.NAV_LEFT.value:
            if wall == "-1:-1":
                return self.no_bumper_left_no_wall()
            else:
                return self.no_bumper_left_wall()
        elif nav == Nav_Event.NAV_RIGHT.value:
            if wall == "-1:-1":
                return self.no_bumper_right_no_wall()
            else:
                return self.no_bumper_right_wall()
        elif nav == Nav_Event.NAV_PASS.value:
            if wall == "-1:-1":
                return self.no_bumper_pass_no_wall()
            else:
                return self.no_bumper_pass_wall()
        elif nav == Nav_Event.NAV_DOCK.value:
            if wall == "-1:-1":
                return self.no_bumper_dock_no_wall()
            else:
                return self.no_bumper_dock_wall()
        else:
            if wall == "-1:-1":
                return self.no_bumper_none_no_wall()
            else:
                return self.no_bumper_none_wall()


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

class Docked(State):
    def __init__(self, actionPublisher):
        super().__init__(actionPublisher)
        self.stateType = StateType.DOCKED

class No_Dest(Operational):
    def __init__(self, actionPublisher):
        super().__init__(actionPublisher)
        self.stateType = StateType.NO_DEST

    def no_bumper_none_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value))
        return self

    def no_bumper_none_wall(self):
        self.actionPublisher.publish(generateAction(Action.WALL_FOLLOW.value, self.wall))
        return self
    
    def no_bumper_left_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value))
        return Should_Turn_Left(self.actionPublisher)

    def no_bumper_left_wall(self):
        self.actionPublisher.publish(generateAction(Action.WALL_FOLLOW.value, self.wall))
        return Should_Turn_Left(self.actionPublisher)
    
    def no_bumper_right_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value))
        return Should_Turn_Right(self.actionPublisher)

    def no_bumper_right_wall(self):
        self.actionPublisher.publish(generateAction(Action.WALL_FOLLOW.value, self.wall))
        return Should_Turn_Right(self.actionPublisher)
    
    def no_bumper_pass_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value))
        return Should_Pass(self.actionPublisher)

    def no_bumper_pass_wall(self):
        self.actionPublisher.publish(generateAction(Action.WALL_FOLLOW.value, self.wall))
        return Should_Pass(self.actionPublisher)

    def no_bumper_dock_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value))
        return Should_Dock(self.actionPublisher)

    def no_bumper_dock_wall(self):
        self.actionPublisher.publish(generateAction(Action.WALL_FOLLOW.value, self.wall))
        return Should_Dock(self.actionPublisher)
    
    def bumper_none(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        return Collision_No_Dest(self.actionPublisher)

    def bumper_left(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        return Collision_Turn_Left(self.actionPublisher)
    
    def bumper_right(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        return Collision_Turn_Right(self.actionPublisher)

    def bumper_pass(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        return Collision_Pass(self.actionPublisher)
    
    def bumper_dock(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        return Collision_Dock(self.actionPublisher)
    

class Should_Turn_Left(Operational):
    def __init__(self, actionPublisher):
        super().__init__(actionPublisher)
        self.stateType = StateType.SHOULD_TURN_LEFT

    def no_bumper_none_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        return Handle_Intersection(self.actionPublisher)

    def no_bumper_none_wall(self):
        self.actionPublisher.publish(generateAction(Action.WALL_FOLLOW.value, self.wall))
        return self
    
    def no_bumper_left_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        return Handle_Intersection(self.actionPublisher)

    def no_bumper_left_wall(self):
        self.actionPublisher.publish(generateAction(Action.WALL_FOLLOW.value, self.wall))
        return self
    
    def no_bumper_right_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        return Handle_Intersection(self.actionPublisher)

    def no_bumper_right_wall(self):
        self.actionPublisher.publish(generateAction(Action.WALL_FOLLOW.value, self.wall))
        return self
    
    def no_bumper_pass_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        return Handle_Intersection(self.actionPublisher)

    def no_bumper_pass_wall(self):
        self.actionPublisher.publish(generateAction(Action.WALL_FOLLOW.value, self.wall))
        return self

    def no_bumper_dock_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        return Handle_Intersection(self.actionPublisher)

    def no_bumper_dock_wall(self):
        self.actionPublisher.publish(generateAction(Action.WALL_FOLLOW.value, self.wall))
        return self
    
    def bumper_none(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        return Collision_Turn_Left(self.actionPublisher)

    def bumper_left(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        return Collision_Turn_Left(self.actionPublisher)
    
    def bumper_right(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        return Collision_Turn_Left(self.actionPublisher)

    def bumper_pass(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        return Collision_Turn_Left(self.actionPublisher)
    
    def bumper_dock(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        return Collision_Turn_Left(self.actionPublisher)

class Should_Turn_Right(Operational):
    def __init__(self, actionPublisher):
        super().__init__(actionPublisher)
        self.stateType = StateType.SHOULD_TURN_RIGHT  

    def no_bumper_none_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value))
        return Handle_Intersection(self.actionPublisher)

    def no_bumper_none_wall(self):
        self.actionPublisher.publish(generateAction(Action.WALL_FOLLOW.value, self.wall))
        return self
    
    def no_bumper_left_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value))
        return Handle_Intersection(self.actionPublisher)

    def no_bumper_left_wall(self):
        self.actionPublisher.publish(generateAction(Action.WALL_FOLLOW.value, self.wall))
        return self
    
    def no_bumper_right_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value))
        return Handle_Intersection(self.actionPublisher)

    def no_bumper_right_wall(self):
        self.actionPublisher.publish(generateAction(Action.WALL_FOLLOW.value, self.wall))
        return self
    
    def no_bumper_pass_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value))
        return Handle_Intersection(self.actionPublisher)

    def no_bumper_pass_wall(self):
        self.actionPublisher.publish(generateAction(Action.WALL_FOLLOW.value, self.wall))
        return self

    def no_bumper_dock_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value))
        return Handle_Intersection(self.actionPublisher)

    def no_bumper_dock_wall(self):
        self.actionPublisher.publish(generateAction(Action.WALL_FOLLOW.value, self.wall))
        return self
    
    def bumper_none(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        return Collision_Turn_Right(self.actionPublisher)

    def bumper_left(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        return Collision_Turn_Right(self.actionPublisher)
    
    def bumper_right(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        return Collision_Turn_Right(self.actionPublisher)

    def bumper_pass(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        return Collision_Turn_Right(self.actionPublisher)
    
    def bumper_dock(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        return Collision_Turn_Right(self.actionPublisher)

class Should_Pass(Operational):
    def __init__(self, actionPublisher):
        super().__init__(actionPublisher)
        self.stateType = StateType.SHOULD_PASS

    def no_bumper_none_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.FORWARD.value))
        return Handle_Intersection(self.actionPublisher)

    def no_bumper_none_wall(self):
        self.actionPublisher.publish(generateAction(Action.WALL_FOLLOW.value, self.wall))
        return self
    
    def no_bumper_left_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.FORWARD.value))
        return Handle_Intersection(self.actionPublisher)

    def no_bumper_left_wall(self):
        self.actionPublisher.publish(generateAction(Action.WALL_FOLLOW.value, self.wall))
        return self
    
    def no_bumper_right_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.FORWARD.value))
        return Handle_Intersection(self.actionPublisher)

    def no_bumper_right_wall(self):
        self.actionPublisher.publish(generateAction(Action.WALL_FOLLOW.value, self.wall))
        return self
    
    def no_bumper_pass_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.FORWARD.value))
        return Handle_Intersection(self.actionPublisher)

    def no_bumper_pass_wall(self):
        self.actionPublisher.publish(generateAction(Action.WALL_FOLLOW.value, self.wall))
        return self

    def no_bumper_dock_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.FORWARD.value))
        return Handle_Intersection(self.actionPublisher)

    def no_bumper_dock_wall(self):
        self.actionPublisher.publish(generateAction(Action.WALL_FOLLOW.value, self.wall))
        return self
    
    def bumper_none(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        return Collision_Pass(self.actionPublisher)

    def bumper_left(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        return Collision_Pass(self.actionPublisher)
    
    def bumper_right(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        return Collision_Pass(self.actionPublisher)

    def bumper_pass(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        return Collision_Pass(self.actionPublisher)
    
    def bumper_dock(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        return Collision_Pass(self.actionPublisher)

class Should_Dock(Operational):
    def __init__(self, actionPublisher):
        super().__init__(actionPublisher)
        self.stateType = StateType.SHOULD_DOCK

    def no_bumper_none_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.DOCK.value))
        return Docked(self.actionPublisher)

    def no_bumper_none_wall(self):
        self.actionPublisher.publish(generateAction(Action.WALL_FOLLOW.value, self.wall))
        return self
    
    def no_bumper_left_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.DOCK.value))
        return Docked(self.actionPublisher)

    def no_bumper_left_wall(self):
        self.actionPublisher.publish(generateAction(Action.WALL_FOLLOW.value, self.wall))
        return self
    
    def no_bumper_right_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.DOCK.value))
        return Docked(self.actionPublisher)

    def no_bumper_right_wall(self):
        self.actionPublisher.publish(generateAction(Action.WALL_FOLLOW.value, self.wall))
        return self
    
    def no_bumper_pass_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.DOCK.value))
        return Docked(self.actionPublisher)

    def no_bumper_pass_wall(self):
        self.actionPublisher.publish(generateAction(Action.WALL_FOLLOW.value, self.wall))
        return self

    def no_bumper_dock_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.DOCK.value))
        return Docked(self.actionPublisher)

    def no_bumper_dock_wall(self):
        self.actionPublisher.publish(generateAction(Action.WALL_FOLLOW.value, self.wall))
        return self
    
    def bumper_none(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        return Collision_Dock(self.actionPublisher)

    def bumper_left(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        return Collision_Dock(self.actionPublisher)
    
    def bumper_right(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        return Collision_Dock(self.actionPublisher)

    def bumper_pass(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        return Collision_Dock(self.actionPublisher)
    
    def bumper_dock(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        return Collision_Dock(self.actionPublisher)

class Handle_Intersection(Operational):
    def __init__(self, actionPublisher):
        super().__init__(actionPublisher)
        self.stateType = StateType.HANDLE_INTERSECTION

    def no_bumper_none_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.FORWARD.value))
        return self

    def no_bumper_none_wall(self):
        self.actionPublisher.publish(generateAction(Action.WALL_FOLLOW.value, self.wall))
        return No_Dest(self.actionPublisher)
    
    def no_bumper_left_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.FORWARD.value))
        return self

    def no_bumper_left_wall(self):
        self.actionPublisher.publish(generateAction(Action.WALL_FOLLOW.value, self.wall))
        return No_Dest(self.actionPublisher)
    
    def no_bumper_right_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.FORWARD.value))
        return self

    def no_bumper_right_wall(self):
        self.actionPublisher.publish(generateAction(Action.WALL_FOLLOW.value, self.wall))
        return No_Dest(self.actionPublisher)
    
    def no_bumper_pass_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.FORWARD.value))
        return self

    def no_bumper_pass_wall(self):
        self.actionPublisher.publish(generateAction(Action.WALL_FOLLOW.value, self.wall))
        return No_Dest(self.actionPublisher)

    def no_bumper_dock_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.FORWARD.value))
        return self

    def no_bumper_dock_wall(self):
        self.actionPublisher.publish(generateAction(Action.WALL_FOLLOW.value, self.wall))
        return No_Dest(self.actionPublisher)
    
    def bumper_none(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        return Collision_Intersection(self.actionPublisher)

    def bumper_left(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        return Collision_Intersection(self.actionPublisher)
    
    def bumper_right(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        return Collision_Intersection(self.actionPublisher)

    def bumper_pass(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        return Collision_Intersection(self.actionPublisher)
    
    def bumper_dock(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        return Collision_Dock(self.actionPublisher)

class Collision_No_Dest(Operational):
    def __init__(self, actionPublisher, count = 1):
        super().__init__(actionPublisher)
        self.stateType = StateType.COLLISION_NO_DEST
        self.count = count

    def no_bumper_none_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return No_Dest(self.actionPublisher)

    def no_bumper_none_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return No_Dest(self.actionPublisher)
    
    def no_bumper_left_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Should_Turn_Left(self.actionPublisher)

    def no_bumper_left_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Should_Turn_Left(self.actionPublisher)
    
    def no_bumper_right_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Should_Turn_Right(self.actionPublisher)

    def no_bumper_right_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Should_Turn_Right(self.actionPublisher)
    
    def no_bumper_pass_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Should_Pass(self.actionPublisher)

    def no_bumper_pass_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Should_Pass(self.actionPublisher)

    def no_bumper_dock_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Should_Dock(self.actionPublisher)

    def no_bumper_dock_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Should_Dock(self.actionPublisher)
    
    def bumper_none(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        self.count += 1
        return self

    def bumper_left(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        self.count += 1
        return Collision_Turn_Left(self.actionPublisher, self.count)
    
    def bumper_right(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        self.count += 1
        return Collision_Turn_Right(self.actionPublisher, self.count)

    def bumper_pass(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        self.count += 1
        return Collision_Pass(self.actionPublisher, self.count)
    
    def bumper_dock(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        self.count += 1
        return Collision_Dock(self.actionPublisher, self.count)

class Collision_Turn_Left(Operational):
    def __init__(self, actionPublisher, count = 1):
        super().__init__(actionPublisher)
        self.stateType = StateType.COLLISION_TURN_LEFT
        self.count = count

    def no_bumper_none_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Should_Turn_Left(self.actionPublisher)

    def no_bumper_none_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Should_Turn_Left(self.actionPublisher)
    
    def no_bumper_left_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Should_Turn_Left(self.actionPublisher)

    def no_bumper_left_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Should_Turn_Left(self.actionPublisher)
    
    def no_bumper_right_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Should_Turn_Left(self.actionPublisher)

    def no_bumper_right_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Should_Turn_Left(self.actionPublisher)
    
    def no_bumper_pass_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Should_Turn_Left(self.actionPublisher)

    def no_bumper_pass_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Should_Turn_Left(self.actionPublisher)

    def no_bumper_dock_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Should_Turn_Left(self.actionPublisher)

    def no_bumper_dock_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Should_Turn_Left(self.actionPublisher)
    
    def bumper_none(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        self.count += 1
        return self

    def bumper_left(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        self.count += 1
        return self
    
    def bumper_right(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        self.count += 1
        return self

    def bumper_pass(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        self.count += 1
        return self
    
    def bumper_dock(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        self.count += 1
        return self

class Collision_Turn_Right(Operational):
    def __init__(self, actionPublisher, count = 1):
        super().__init__(actionPublisher)
        self.stateType = StateType.COLLISION_TURN_RIGHT
        self.count = count

    def no_bumper_none_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Should_Turn_Right(self.actionPublisher)

    def no_bumper_none_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Should_Turn_Right(self.actionPublisher)
    
    def no_bumper_left_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Should_Turn_Right(self.actionPublisher)

    def no_bumper_left_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Should_Turn_Right(self.actionPublisher)
    
    def no_bumper_right_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Should_Turn_Right(self.actionPublisher)

    def no_bumper_right_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Should_Turn_Right(self.actionPublisher)
    
    def no_bumper_pass_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Should_Turn_Right(self.actionPublisher)

    def no_bumper_pass_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Should_Turn_Right(self.actionPublisher)

    def no_bumper_dock_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Should_Turn_Right(self.actionPublisher)

    def no_bumper_dock_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Should_Turn_Right(self.actionPublisher)
    
    def bumper_none(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        self.count += 1
        return self

    def bumper_left(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        self.count += 1
        return self
    
    def bumper_right(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        self.count += 1
        return self

    def bumper_pass(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        self.count += 1
        return self
    
    def bumper_dock(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        self.count += 1
        return self

class Collision_Pass(Operational):
    def __init__(self, actionPublisher, count = 1):
        super().__init__(actionPublisher)
        self.stateType = StateType.COLLISION_PASS
        self.count = count

    def no_bumper_none_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Should_Pass(self.actionPublisher)

    def no_bumper_none_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Should_Pass(self.actionPublisher)
    
    def no_bumper_left_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Should_Pass(self.actionPublisher)

    def no_bumper_left_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Should_Pass(self.actionPublisher)
    
    def no_bumper_right_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Should_Pass(self.actionPublisher)

    def no_bumper_right_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Should_Pass(self.actionPublisher)
    
    def no_bumper_pass_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Should_Pass(self.actionPublisher)

    def no_bumper_pass_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Should_Pass(self.actionPublisher)

    def no_bumper_dock_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Should_Pass(self.actionPublisher)

    def no_bumper_dock_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Should_Pass(self.actionPublisher)
    
    def bumper_none(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        self.count += 1
        return self

    def bumper_left(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        self.count += 1
        return self
    
    def bumper_right(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        self.count += 1
        return self

    def bumper_pass(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        self.count += 1
        return self
    
    def bumper_dock(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        self.count += 1
        return self

class Collision_Dock(Operational):
    def __init__(self, actionPublisher, count = 1):
        super().__init__(actionPublisher)
        self.stateType = StateType.COLLISION_DOCK
        self.count = count

    def no_bumper_none_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Should_Dock(self.actionPublisher)

    def no_bumper_none_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Should_Dock(self.actionPublisher)
    
    def no_bumper_left_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Should_Dock(self.actionPublisher)

    def no_bumper_left_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Should_Dock(self.actionPublisher)
    
    def no_bumper_right_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Should_Dock(self.actionPublisher)

    def no_bumper_right_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Should_Dock(self.actionPublisher)
    
    def no_bumper_pass_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Should_Dock(self.actionPublisher)

    def no_bumper_pass_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Should_Dock(self.actionPublisher)

    def no_bumper_dock_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Should_Dock(self.actionPublisher)

    def no_bumper_dock_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Should_Dock(self.actionPublisher)
    
    def bumper_none(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        self.count += 1
        return self

    def bumper_left(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        self.count += 1
        return self
    
    def bumper_right(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        self.count += 1
        return self

    def bumper_pass(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        self.count += 1
        return self
    
    def bumper_dock(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        self.count += 1
        return self

class Collision_Intersection(Operational):
    def __init__(self, actionPublisher, count = 1):
        super().__init__(actionPublisher)
        self.stateType = StateType.COLLISION_INTERSECTION
        self.count = count

    def no_bumper_none_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Handle_Intersection(self.actionPublisher)

    def no_bumper_none_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Handle_Intersection(self.actionPublisher)
    
    def no_bumper_left_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Handle_Intersection(self.actionPublisher)

    def no_bumper_left_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Handle_Intersection(self.actionPublisher)
    
    def no_bumper_right_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Handle_Intersection(self.actionPublisher)

    def no_bumper_right_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Handle_Intersection(self.actionPublisher)
    
    def no_bumper_pass_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Handle_Intersection(self.actionPublisher)

    def no_bumper_pass_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Handle_Intersection(self.actionPublisher)

    def no_bumper_dock_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Handle_Intersection(self.actionPublisher)

    def no_bumper_dock_wall(self):
        self.actionPublisher.publish(generateAction(Action.R_TURN.value, self.count))
        return Handle_Intersection(self.actionPublisher)
    
    def bumper_none(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        self.count += 1
        return self

    def bumper_left(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        self.count += 1
        return self
    
    def bumper_right(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        self.count += 1
        return self

    def bumper_pass(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        self.count += 1
        return self
    
    def bumper_dock(self):
        self.actionPublisher.publish(generateAction(Action.L_TURN.value))
        self.count += 1
        return self

def generateAction(command, data = ""):
    '''
    Genrates the appropriate action message based on the given command.

    @param command: The command to create an action for.
    @parm data: The data for the action.
    '''
    action = String()
    action.data = command
    if data != "":
        action.data += ":" + str(data)
    return action
