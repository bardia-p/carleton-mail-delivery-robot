from enum import Enum
from std_msgs.msg import String

from navigation.captain import Nav_Event
from tools.csv_parser import loadConfig

config = loadConfig()
LEFT_TURN_LIMIT = config["LEFT_TURN_CLOCK_CYCLES"]
RIGHT_TURN_LIMIT = config["RIGHT_TURN_CLOCK_CYCLES"]
U_TURN_LIMIT = config["U_TURN_CLOCK_CYCLES"]
FORWARD_LIMIT = config["FORWARD_CLOCK_CYCLES"]
WALL_FOLLOW_LIMIT = config["WALL_FOLLOW_CLOCK_CYCLES"]
LEFT_FACTOR = config["LEFT_TURN_MULTI_FACTOR"]
RIGHT_FACTOR = config["RIGHT_TURN_MULTI_FACTOR"]
FORWARD_FACTOR = config["FORWARD_MULTI_FACTOR"]
COLLISION_FACTOR = config["COLLISION_MULTI_FACTOR"]


class StateType(Enum):
    OPERATIONAL = "OPERATIONAL"
    NOT_OPERATIONAL = "NOT_OPERATIONAL"
    DOCKED = "DOCKED"
    NO_DEST = "NO_DEST"
    SHOULD_TURN_LEFT = "SHOULD_TURN_LEFT"
    SHOULD_TURN_RIGHT = "SHOULD_TURN_RIGHT"
    SHOULD_U_TURN = "SHOULD_U_TURN"
    SHOULD_PASS = "SHOULD_PASS"
    SHOULD_DOCK = "SHOULD_DOCK"
    HANDLE_INTERSECTION = "HANDLE_INTERSECTION"
    COLLISION_NO_DEST = "COLLISION_NO_DEST"
    COLLISION_TURN_LEFT = "COLLISION_TURN_LEFT"
    COLLISION_TURN_RIGHT = "COLLISION_TURN_RIGHT"
    COLLISION_PASS = "COLLISION_PASS"
    COLLISION_DOCK = "COLLISION_DOCK"
    COLLISION_INTERSECTION = "COLLISION_INTERSECTION"
    COLLISION_U_TURN = "COLLISION_U_TURN"


class Action(Enum):
    L_TURN = "L_TURN"
    U_TURN = "U_TURN"
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
        self.nav = None
        self.isBusy = False
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

    def no_bumper_uturn_no_wall(self):
        return self

    def no_bumper_uturn_wall(self):
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

    def bumper_uturn(self):
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
        self.wall = wall
        # Make sure to capture important navigation info.
        if nav != "NAV_NONE":
            self.nav = nav

        # Checks to see if the state machine has unfinished action.
        if self.longAction != None:
            if self.longActionCount < self.longActionLimit:
                self.actionPublisher.publish(self.longAction)
                self.longActionCount += 1
                return self
            else:
                self.longActionCount = 0
                self.longActionLimit = 0
                self.longAction = None
                self.isBusy = False
                return self.postLongActionState

        # Finds the proper transition based on the data.

        # Restore the saved nav data.
        if self.nav != "NAV_NONE":
            nav = self.nav
            self.nav = "NAV_NONE"

        if bumper:
            if nav == Nav_Event.NAV_LEFT.value:
                return self.bumper_left()
            elif nav == Nav_Event.NAV_RIGHT.value:
                return self.bumper_right()
            elif nav == Nav_Event.NAV_PASS.value:
                return self.bumper_pass()
            elif nav == Nav_Event.NAV_DOCK.value:
                return self.bumper_dock()
            elif nav == Nav_Event.NAV_U_TURN.value:
                return self.bumper_uturn()
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
        elif nav == Nav_Event.NAV_U_TURN.value:
            if wall == "-1:-1":
                return self.no_bumper_uturn_no_wall()
            else:
                return self.no_bumper_uturn_wall()

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

    def setLongAction(self, longAction, longActionLimit, postLongActionState=None):
        '''
        Defines an action for the state machine that can span multiple clock cycles.

        @param longAction: The action to generate.
        @param longActionLimit: The number of cycles required for the action.
        '''
        self.isBusy = True
        self.longAction = longAction
        self.longActionCount = 1
        self.longActionLimit = longActionLimit
        self.postLongActionState = self if postLongActionState == None else postLongActionState
        self.actionPublisher.publish(self.longAction)


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
        self.noWallCount = 0

    def no_bumper_none_no_wall(self):
        if self.noWallCount % 2 == 0:
            self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT)
        else:
            self.setLongAction(generateAction(Action.FORWARD.value), FORWARD_LIMIT)
        self.noWallCount += 1
        return self

    def no_bumper_none_wall(self):
        self.setLongAction(generateAction(Action.WALL_FOLLOW.value, self.wall), WALL_FOLLOW_LIMIT)
        return self

    def no_bumper_left_no_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT,
                           Should_Turn_Left(self.actionPublisher))
        return self

    def no_bumper_left_wall(self):
        self.setLongAction(generateAction(Action.WALL_FOLLOW.value, self.wall), WALL_FOLLOW_LIMIT,
                           Should_Turn_Left(self.actionPublisher))
        return self

    def no_bumper_uturn_no_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), U_TURN_LIMIT, Should_U_Turn(self.actionPublisher))
        return self

    def no_bumper_uturn_wall(self):
        self.setLongAction(generateAction(Action.WALL_FOLLOW.value, self.wall), WALL_FOLLOW_LIMIT,
                           Should_U_Turn(self.actionPublisher))
        return self

    def no_bumper_right_no_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT,
                           Should_Turn_Right(self.actionPublisher))
        return self

    def no_bumper_right_wall(self):
        self.setLongAction(generateAction(Action.WALL_FOLLOW.value, self.wall), WALL_FOLLOW_LIMIT, Should_Turn_Right(self.actionPublisher))
        return self

    def no_bumper_pass_no_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT, Should_Pass(self.actionPublisher))
        return self

    def no_bumper_pass_wall(self):
        self.setLongAction(generateAction(Action.WALL_FOLLOW.value, self.wall), WALL_FOLLOW_LIMIT,
                           Should_Pass(self.actionPublisher))
        return self

    def no_bumper_dock_no_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT, Should_Dock(self.actionPublisher))
        return self

    def no_bumper_dock_wall(self):
        self.setLongAction(generateAction(Action.WALL_FOLLOW.value, self.wall), WALL_FOLLOW_LIMIT,
                           Should_Dock(self.actionPublisher))
        return self

    def bumper_none(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT,
                           Collision_No_Dest(self.actionPublisher))
        return self

    def bumper_left(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT,
                           Collision_Turn_Left(self.actionPublisher))
        return self

    def bumper_uturn(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT, Collision_U_Turn(self.actionPublisher))
        return self

    def bumper_right(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT,
                           Collision_Turn_Right(self.actionPublisher))
        return self

    def bumper_pass(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT, Collision_Pass(self.actionPublisher))
        return self

    def bumper_dock(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT, Collision_Dock(self.actionPublisher))
        return self


class Should_Turn_Left(Operational):
    def __init__(self, actionPublisher):
        super().__init__(actionPublisher)
        self.stateType = StateType.SHOULD_TURN_LEFT

    def no_bumper_none_no_wall(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT * LEFT_FACTOR,
                           Handle_Intersection(self.actionPublisher))
        return self

    def no_bumper_none_wall(self):
        self.setLongAction(generateAction(Action.WALL_FOLLOW.value, self.wall), WALL_FOLLOW_LIMIT)
        return self

    def no_bumper_left_no_wall(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT * LEFT_FACTOR,
                           Handle_Intersection(self.actionPublisher))
        return self

    def no_bumper_left_wall(self):
        self.setLongAction(generateAction(Action.WALL_FOLLOW.value, self.wall), WALL_FOLLOW_LIMIT)
        return self

    def no_bumper_uturn_no_wall(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT, Handle_Intersection(self.actionPublisher))
        return self

    def no_bumper_uturn_wall(self):
        self.setLongAction(generateAction(Action.WALL_FOLLOW.value, self.wall), WALL_FOLLOW_LIMIT)
        return self

    def no_bumper_right_no_wall(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT * LEFT_FACTOR,
                           Handle_Intersection(self.actionPublisher))
        return self

    def no_bumper_right_wall(self):
        self.setLongAction(generateAction(Action.WALL_FOLLOW.value, self.wall), WALL_FOLLOW_LIMIT)
        return self

    def no_bumper_pass_no_wall(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT * LEFT_FACTOR,
                           Handle_Intersection(self.actionPublisher))
        return self

    def no_bumper_pass_wall(self):
        self.setLongAction(generateAction(Action.WALL_FOLLOW.value, self.wall), WALL_FOLLOW_LIMIT)
        return self

    def no_bumper_dock_no_wall(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT * LEFT_FACTOR,
                           Handle_Intersection(self.actionPublisher))
        return self

    def no_bumper_dock_wall(self):
        self.setLongAction(generateAction(Action.WALL_FOLLOW.value, self.wall), WALL_FOLLOW_LIMIT)
        return self

    def bumper_none(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT,
                           Collision_Turn_Left(self.actionPublisher))
        return self

    def bumper_left(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT,
                           Collision_Turn_Left(self.actionPublisher))
        return self

    def bumper_uturn(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT, Collision_Turn_Left(self.actionPublisher))
        return self

    def bumper_right(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT,
                           Collision_Turn_Left(self.actionPublisher))
        return self

    def bumper_pass(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT,
                           Collision_Turn_Left(self.actionPublisher))
        return self

    def bumper_dock(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT,
                           Collision_Turn_Left(self.actionPublisher))
        return self

class Should_U_Turn(Operational):
    def __init__(self, actionPublisher):
        super().__init__(actionPublisher)
        self.stateType = StateType.SHOULD_U_TURN

    def no_bumper_none_no_wall(self):
        self.setLongAction(generateAction(Action.U_TURN.value), U_TURN_LIMIT,
                           Handle_Intersection(self.actionPublisher))
        return self

    def no_bumper_none_wall(self):
        self.setLongAction(generateAction(Action.WALL_FOLLOW.value, self.wall), WALL_FOLLOW_LIMIT)
        return self

    def no_bumper_left_no_wall(self):
        self.setLongAction(generateAction(Action.U_TURN.value), U_TURN_LIMIT,
                           Handle_Intersection(self.actionPublisher))
        return self

    def no_bumper_left_wall(self):
        self.setLongAction(generateAction(Action.WALL_FOLLOW.value, self.wall), WALL_FOLLOW_LIMIT)
        return self

    def no_bumper_uturn_no_wall(self):
        self.setLongAction(generateAction(Action.U_TURN.value), U_TURN_LIMIT,
                           Handle_Intersection(self.actionPublisher))
        return self

    def no_bumper_uturn_wall(self):
        self.setLongAction(generateAction(Action.WALL_FOLLOW.value, self.wall), WALL_FOLLOW_LIMIT)
        return self

    def no_bumper_right_no_wall(self):
        self.setLongAction(generateAction(Action.U_TURN.value), U_TURN_LIMIT,
                           Handle_Intersection(self.actionPublisher))
        return self

    def no_bumper_right_wall(self):
        self.setLongAction(generateAction(Action.WALL_FOLLOW.value, self.wall), WALL_FOLLOW_LIMIT)
        return self

    def no_bumper_pass_no_wall(self):
        self.setLongAction(generateAction(Action.U_TURN.value), U_TURN_LIMIT,
                           Handle_Intersection(self.actionPublisher))
        return self

    def no_bumper_pass_wall(self):
        self.setLongAction(generateAction(Action.WALL_FOLLOW.value, self.wall), WALL_FOLLOW_LIMIT)
        return self

    def no_bumper_dock_no_wall(self):
        self.setLongAction(generateAction(Action.U_TURN.value), U_TURN_LIMIT,
                           Handle_Intersection(self.actionPublisher))
        return self

    def no_bumper_dock_wall(self):
        self.setLongAction(generateAction(Action.WALL_FOLLOW.value, self.wall), WALL_FOLLOW_LIMIT)
        return self

    def bumper_none(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT,
                           Collision_U_Turn(self.actionPublisher))
        return self

    def bumper_left(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT,
                           Collision_U_Turn(self.actionPublisher))
        return self

    def bumper_uturn(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT,
                           Collision_U_Turn(self.actionPublisher))
        return self

    def bumper_right(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT,
                           Collision_U_Turn(self.actionPublisher))
        return self

    def bumper_pass(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT,
                           Collision_U_Turn(self.actionPublisher))
        return self

    def bumper_dock(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT,
                           Collision_U_Turn(self.actionPublisher))
        return self

class Should_Turn_Right(Operational):
    def __init__(self, actionPublisher):
        super().__init__(actionPublisher)
        self.stateType = StateType.SHOULD_TURN_RIGHT

    def no_bumper_none_no_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * RIGHT_FACTOR,
                           Handle_Intersection(self.actionPublisher))
        return self

    def no_bumper_none_wall(self):
        self.setLongAction(generateAction(Action.WALL_FOLLOW.value, self.wall), WALL_FOLLOW_LIMIT)
        return self

    def no_bumper_left_no_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * RIGHT_FACTOR,
                           Handle_Intersection(self.actionPublisher))
        return self

    def no_bumper_left_wall(self):
        self.setLongAction(generateAction(Action.WALL_FOLLOW.value, self.wall), WALL_FOLLOW_LIMIT)
        return self

    def no_bumper_uturn_no_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT, Handle_Intersection(self.actionPublisher))
        return self

    def no_bumper_uturn_wall(self):
        self.setLongAction(generateAction(Action.WALL_FOLLOW.value, self.wall), WALL_FOLLOW_LIMIT)
        return self

    def no_bumper_right_no_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * RIGHT_FACTOR,
                           Handle_Intersection(self.actionPublisher))
        return self

    def no_bumper_right_wall(self):
        self.setLongAction(generateAction(Action.WALL_FOLLOW.value, self.wall), WALL_FOLLOW_LIMIT)
        return self

    def no_bumper_pass_no_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * RIGHT_FACTOR,
                           Handle_Intersection(self.actionPublisher))
        return self

    def no_bumper_pass_wall(self):
        self.setLongAction(generateAction(Action.WALL_FOLLOW.value, self.wall), WALL_FOLLOW_LIMIT)
        return self

    def no_bumper_dock_no_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * RIGHT_FACTOR,
                           Handle_Intersection(self.actionPublisher))
        return self

    def no_bumper_dock_wall(self):
        self.setLongAction(generateAction(Action.WALL_FOLLOW.value, self.wall), WALL_FOLLOW_LIMIT)
        return self

    def bumper_none(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT,
                           Collision_Turn_Right(self.actionPublisher))
        return self

    def bumper_uturn(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT, Collision_Turn_Right(self.actionPublisher))
        return self

    def bumper_left(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT,
                           Collision_Turn_Right(self.actionPublisher))
        return self

    def bumper_right(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT,
                           Collision_Turn_Right(self.actionPublisher))
        return self

    def bumper_pass(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT,
                           Collision_Turn_Right(self.actionPublisher))
        return self

    def bumper_dock(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT,
                           Collision_Turn_Right(self.actionPublisher))
        return self


class Should_Pass(Operational):
    def __init__(self, actionPublisher):
        super().__init__(actionPublisher)
        self.stateType = StateType.SHOULD_PASS

    def no_bumper_none_no_wall(self):
        self.setLongAction(generateAction(Action.FORWARD.value), FORWARD_LIMIT * FORWARD_FACTOR,
                           Handle_Intersection(self.actionPublisher))
        return self

    def no_bumper_none_wall(self):
        self.setLongAction(generateAction(Action.WALL_FOLLOW.value, self.wall), WALL_FOLLOW_LIMIT)
        return self

    def no_bumper_left_no_wall(self):
        self.setLongAction(generateAction(Action.FORWARD.value), FORWARD_LIMIT * FORWARD_FACTOR,
                           Handle_Intersection(self.actionPublisher))
        return self

    def no_bumper_left_wall(self):
        self.setLongAction(generateAction(Action.WALL_FOLLOW.value, self.wall), WALL_FOLLOW_LIMIT)
        return self

    def no_bumper_uturn_no_wall(self):
        self.setLongAction(generateAction(Action.FORWARD.value), FORWARD_LIMIT, Handle_Intersection(self.actionPublisher))
        return self

    def no_bumper_uturn_wall(self):
        self.setLongAction(generateAction(Action.WALL_FOLLOW.value, self.wall), WALL_FOLLOW_LIMIT)
        return self

    def no_bumper_right_no_wall(self):
        self.setLongAction(generateAction(Action.FORWARD.value), FORWARD_LIMIT * FORWARD_FACTOR,
                           Handle_Intersection(self.actionPublisher))
        return self

    def no_bumper_right_wall(self):
        self.setLongAction(generateAction(Action.WALL_FOLLOW.value, self.wall), WALL_FOLLOW_LIMIT)
        return self

    def no_bumper_pass_no_wall(self):
        self.setLongAction(generateAction(Action.FORWARD.value), FORWARD_LIMIT * FORWARD_FACTOR,
                           Handle_Intersection(self.actionPublisher))
        return self

    def no_bumper_pass_wall(self):
        self.setLongAction(generateAction(Action.WALL_FOLLOW.value, self.wall), WALL_FOLLOW_LIMIT)
        return self

    def no_bumper_dock_no_wall(self):
        self.setLongAction(generateAction(Action.FORWARD.value), FORWARD_LIMIT * FORWARD_FACTOR,
                           Handle_Intersection(self.actionPublisher))
        return self

    def no_bumper_dock_wall(self):
        self.setLongAction(generateAction(Action.WALL_FOLLOW.value, self.wall), WALL_FOLLOW_LIMIT)
        return self

    def bumper_none(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT, Collision_Pass(self.actionPublisher))
        return self

    def bumper_left(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT, Collision_Pass(self.actionPublisher))
        return self

    def bumper_uturn(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT, Collision_Pass(self.actionPublisher))
        return self

    def bumper_right(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT, Collision_Pass(self.actionPublisher))
        return self

    def bumper_pass(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT, Collision_Pass(self.actionPublisher))
        return self

    def bumper_dock(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT, Collision_Pass(self.actionPublisher))
        return self


class Should_Dock(Operational):
    def __init__(self, actionPublisher):
        super().__init__(actionPublisher)
        self.stateType = StateType.SHOULD_DOCK

    def no_bumper_none_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.DOCK.value))
        return Docked(self.actionPublisher)

    def no_bumper_none_wall(self):
        self.setLongAction(generateAction(Action.WALL_FOLLOW.value, self.wall), WALL_FOLLOW_LIMIT)
        return self

    def no_bumper_left_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.DOCK.value))
        return Docked(self.actionPublisher)

    def no_bumper_left_wall(self):
        self.setLongAction(generateAction(Action.WALL_FOLLOW.value, self.wall), WALL_FOLLOW_LIMIT)
        return self

    def no_bumper_uturn_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.DOCK.value))
        return Docked(self.actionPublisher)

    def no_bumper_uturn_wall(self):
        self.setLongAction(generateAction(Action.WALL_FOLLOW.value, self.wall), WALL_FOLLOW_LIMIT)
        return self

    def no_bumper_right_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.DOCK.value))
        return Docked(self.actionPublisher)

    def no_bumper_right_wall(self):
        self.setLongAction(generateAction(Action.WALL_FOLLOW.value, self.wall), WALL_FOLLOW_LIMIT)
        return self

    def no_bumper_pass_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.DOCK.value))
        return Docked(self.actionPublisher)

    def no_bumper_pass_wall(self):
        self.setLongAction(generateAction(Action.WALL_FOLLOW.value, self.wall), WALL_FOLLOW_LIMIT)
        return self

    def no_bumper_dock_no_wall(self):
        self.actionPublisher.publish(generateAction(Action.DOCK.value))
        return Docked(self.actionPublisher)

    def no_bumper_dock_wall(self):
        self.setLongAction(generateAction(Action.WALL_FOLLOW.value, self.wall), WALL_FOLLOW_LIMIT)
        return self

    def bumper_none(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT, Collision_Dock(self.actionPublisher))
        return self

    def bumper_left(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT, Collision_Dock(self.actionPublisher))
        return self

    def bumper_uturn(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT, Collision_Dock(self.actionPublisher))
        return self

    def bumper_right(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT, Collision_Dock(self.actionPublisher))
        return self

    def bumper_pass(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT, Collision_Dock(self.actionPublisher))
        return self

    def bumper_dock(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT, Collision_Dock(self.actionPublisher))
        return self


class Handle_Intersection(Operational):
    def __init__(self, actionPublisher):
        super().__init__(actionPublisher)
        self.stateType = StateType.HANDLE_INTERSECTION

    def no_bumper_none_no_wall(self):
        self.setLongAction(generateAction(Action.FORWARD.value), FORWARD_LIMIT * FORWARD_FACTOR)
        return self

    def no_bumper_none_wall(self):
        self.setLongAction(generateAction(Action.WALL_FOLLOW.value, self.wall), WALL_FOLLOW_LIMIT,
                           No_Dest(self.actionPublisher))
        return self

    def no_bumper_left_no_wall(self):
        self.setLongAction(generateAction(Action.FORWARD.value), FORWARD_LIMIT * FORWARD_FACTOR)
        return self

    def no_bumper_left_wall(self):
        self.setLongAction(generateAction(Action.WALL_FOLLOW.value, self.wall), WALL_FOLLOW_LIMIT, No_Dest(self.actionPublisher))
        return self

    def no_bumper_uturn_no_wall(self):
        self.setLongAction(generateAction(Action.FORWARD.value), FORWARD_LIMIT * FORWARD_FACTOR)
        return self

    def no_bumper_uturn_wall(self):
        self.setLongAction(generateAction(Action.WALL_FOLLOW.value, self.wall), WALL_FOLLOW_LIMIT, No_Dest(self.actionPublisher))
        return self

    def no_bumper_right_no_wall(self):
        self.setLongAction(generateAction(Action.FORWARD.value), FORWARD_LIMIT * FORWARD_FACTOR)
        return self

    def no_bumper_right_wall(self):
        self.setLongAction(generateAction(Action.WALL_FOLLOW.value, self.wall), WALL_FOLLOW_LIMIT,
                           No_Dest(self.actionPublisher))
        return self

    def no_bumper_pass_no_wall(self):
        self.setLongAction(generateAction(Action.FORWARD.value), FORWARD_LIMIT * FORWARD_FACTOR)
        return self

    def no_bumper_pass_wall(self):
        self.setLongAction(generateAction(Action.WALL_FOLLOW.value, self.wall), WALL_FOLLOW_LIMIT,
                           No_Dest(self.actionPublisher))
        return self

    def no_bumper_dock_no_wall(self):
        self.setLongAction(generateAction(Action.FORWARD.value), FORWARD_LIMIT * FORWARD_FACTOR)
        return self

    def no_bumper_dock_wall(self):
        self.setLongAction(generateAction(Action.WALL_FOLLOW.value, self.wall), WALL_FOLLOW_LIMIT,
                           No_Dest(self.actionPublisher))
        return self

    def bumper_none(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT,
                           Collision_Intersection(self.actionPublisher))
        return self

    def bumper_left(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT,
                           Collision_Intersection(self.actionPublisher))
        return self

    def bumper_uturn(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT, Collision_Intersection(self.actionPublisher))
        return self

    def bumper_right(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT,
                           Collision_Intersection(self.actionPublisher))
        return self

    def bumper_pass(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT,
                           Collision_Intersection(self.actionPublisher))
        return self

    def bumper_dock(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT,
                           Collision_Intersection(self.actionPublisher))
        return self


class Collision_No_Dest(Operational):
    def __init__(self, actionPublisher, count=1):
        super().__init__(actionPublisher)
        self.stateType = StateType.COLLISION_NO_DEST
        self.count = count

    def no_bumper_none_no_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           No_Dest(self.actionPublisher))
        return self

    def no_bumper_none_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           No_Dest(self.actionPublisher))
        return self

    def no_bumper_left_no_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_Turn_Left(self.actionPublisher))
        return self

    def no_bumper_left_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_Turn_Left(self.actionPublisher))
        return self

    def no_bumper_uturn_no_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_U_Turn(self.actionPublisher))
        return self

    def no_bumper_uturn_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_U_Turn(self.actionPublisher))
        return self

    def no_bumper_right_no_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_Turn_Right(self.actionPublisher))
        return self

    def no_bumper_right_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_Turn_Right(self.actionPublisher))
        return self

    def no_bumper_pass_no_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_Pass(self.actionPublisher))
        return self

    def no_bumper_pass_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_Pass(self.actionPublisher))
        return self

    def no_bumper_dock_no_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_Dock(self.actionPublisher))
        return self

    def no_bumper_dock_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_Dock(self.actionPublisher))
        return self

    def bumper_none(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT)
        self.count += 1
        return self

    def bumper_left(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT,
                           Collision_Turn_Left(self.actionPublisher, self.count))
        self.count += 1
        return self

    def bumper_uturn(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT,
                           Collision_U_Turn(self.actionPublisher, self.count))
        self.count += 1
        return self

    def bumper_right(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT,
                           Collision_Turn_Right(self.actionPublisher, self.count))
        self.count += 1
        return self

    def bumper_pass(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT,
                           Collision_Pass(self.actionPublisher, self.count))
        self.count += 1
        return self

    def bumper_dock(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT,
                           Collision_Dock(self.actionPublisher, self.count))
        self.count += 1
        return self


class Collision_Turn_Left(Operational):
    def __init__(self, actionPublisher, count=1):
        super().__init__(actionPublisher)
        self.stateType = StateType.COLLISION_TURN_LEFT
        self.count = count

    def no_bumper_none_no_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_Turn_Left(self.actionPublisher))
        return self

    def no_bumper_none_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_Turn_Left(self.actionPublisher))
        return self

    def no_bumper_left_no_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_Turn_Left(self.actionPublisher))
        return self

    def no_bumper_left_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_Turn_Left(self.actionPublisher))
        return self

    def no_bumper_uturn_no_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR, Should_Turn_Left(self.actionPublisher))
        return self

    def no_bumper_uturn_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR, Should_Turn_Left(self.actionPublisher))
        return self

    def no_bumper_right_no_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_Turn_Left(self.actionPublisher))
        return self

    def no_bumper_right_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_Turn_Left(self.actionPublisher))
        return self

    def no_bumper_pass_no_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_Turn_Left(self.actionPublisher))
        return self

    def no_bumper_pass_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_Turn_Left(self.actionPublisher))
        return self

    def no_bumper_dock_no_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_Turn_Left(self.actionPublisher))
        return self

    def no_bumper_dock_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_Turn_Left(self.actionPublisher))
        return self

    def bumper_none(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT)
        self.count += 1
        return self

    def bumper_left(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT)
        self.count += 1
        return self

    def bumper_uturn(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT)
        self.count += 1
        return self

    def bumper_right(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT)
        self.count += 1
        return self

    def bumper_pass(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT)
        self.count += 1
        return self

    def bumper_dock(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT)
        self.count += 1
        return self

class Collision_U_Turn(Operational):
    def __init__(self, actionPublisher, count=1):
        super().__init__(actionPublisher)
        self.stateType = StateType.COLLISION_U_TURN
        self.count = count

    def no_bumper_none_no_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_U_Turn(self.actionPublisher))
        return self

    def no_bumper_none_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_U_Turn(self.actionPublisher))
        return self

    def no_bumper_left_no_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_U_Turn(self.actionPublisher))
        return self

    def no_bumper_left_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_U_Turn(self.actionPublisher))
        return self

    def no_bumper_uturn_no_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_U_Turn(self.actionPublisher))
        return self

    def no_bumper_uturn_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_U_Turn(self.actionPublisher))
        return self

    def no_bumper_right_no_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_U_Turn(self.actionPublisher))
        return self

    def no_bumper_right_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_U_Turn(self.actionPublisher))
        return self

    def no_bumper_pass_no_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_U_Turn(self.actionPublisher))
        return self

    def no_bumper_pass_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_U_Turn(self.actionPublisher))
        return self

    def no_bumper_dock_no_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_U_Turn(self.actionPublisher))
        return self

    def no_bumper_dock_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_U_Turn(self.actionPublisher))
        return self

    def bumper_none(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT)
        self.count += 1
        return self

    def bumper_left(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT)
        self.count += 1
        return self

    def bumper_uturn(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT)
        self.count += 1
        return self

    def bumper_right(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT)
        self.count += 1
        return self

    def bumper_pass(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT)
        self.count += 1
        return self

    def bumper_dock(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT)
        self.count += 1
        return self

class Collision_Turn_Right(Operational):
    def __init__(self, actionPublisher, count=1):
        super().__init__(actionPublisher)
        self.stateType = StateType.COLLISION_TURN_RIGHT
        self.count = count

    def no_bumper_none_no_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_Turn_Right(self.actionPublisher))
        return self

    def no_bumper_none_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_Turn_Right(self.actionPublisher))
        return self

    def no_bumper_left_no_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_Turn_Right(self.actionPublisher))
        return self

    def no_bumper_left_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_Turn_Right(self.actionPublisher))
        return self

    def no_bumper_uturn_no_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR, Should_Turn_Right(self.actionPublisher))
        return self

    def no_bumper_uturn_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR, Should_Turn_Right(self.actionPublisher))
        return self

    def no_bumper_right_no_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_Turn_Right(self.actionPublisher))
        return self

    def no_bumper_right_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_Turn_Right(self.actionPublisher))
        return self

    def no_bumper_pass_no_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_Turn_Right(self.actionPublisher))
        return self

    def no_bumper_pass_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_Turn_Right(self.actionPublisher))
        return self

    def no_bumper_dock_no_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_Turn_Right(self.actionPublisher))
        return self

    def no_bumper_dock_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_Turn_Right(self.actionPublisher))
        return self

    def bumper_none(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT)
        self.count += 1
        return self

    def bumper_left(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT)
        self.count += 1
        return self

    def bumper_uturn(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT)
        self.count += 1
        return self

    def bumper_right(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT)
        self.count += 1
        return self

    def bumper_pass(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT)
        self.count += 1
        return self

    def bumper_dock(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT)
        self.count += 1
        return self


class Collision_Pass(Operational):
    def __init__(self, actionPublisher, count=1):
        super().__init__(actionPublisher)
        self.stateType = StateType.COLLISION_PASS
        self.count = count

    def no_bumper_none_no_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_Pass(self.actionPublisher))
        return self

    def no_bumper_none_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_Pass(self.actionPublisher))
        return self

    def no_bumper_left_no_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_Pass(self.actionPublisher))
        return self

    def no_bumper_left_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_Pass(self.actionPublisher))
        return self

    def no_bumper_uturn_no_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR, Should_Pass(self.actionPublisher))
        return self

    def no_bumper_uturn_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR, Should_Pass(self.actionPublisher))
        return self

    def no_bumper_right_no_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_Pass(self.actionPublisher))
        return self

    def no_bumper_right_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_Pass(self.actionPublisher))
        return self

    def no_bumper_pass_no_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_Pass(self.actionPublisher))
        return self

    def no_bumper_pass_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_Pass(self.actionPublisher))
        return self

    def no_bumper_dock_no_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_Pass(self.actionPublisher))
        return self

    def no_bumper_dock_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_Pass(self.actionPublisher))
        return self

    def bumper_none(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT)
        self.count += 1
        return self

    def bumper_left(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT)
        self.count += 1
        return self

    def bumper_uturn(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT)
        self.count += 1
        return self

    def bumper_right(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT)
        self.count += 1
        return self

    def bumper_pass(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT)
        self.count += 1
        return self

    def bumper_dock(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT)
        self.count += 1
        return self


class Collision_Dock(Operational):
    def __init__(self, actionPublisher, count=1):
        super().__init__(actionPublisher)
        self.stateType = StateType.COLLISION_DOCK
        self.count = count

    def no_bumper_none_no_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_Dock(self.actionPublisher))
        return self

    def no_bumper_none_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_Dock(self.actionPublisher))
        return self

    def no_bumper_left_no_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_Dock(self.actionPublisher))
        return self

    def no_bumper_left_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_Dock(self.actionPublisher))
        return self

    def no_bumper_uturn_no_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR, Should_Dock(self.actionPublisher))
        return self

    def no_bumper_uturn_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR, Should_Dock(self.actionPublisher))
        return self

    def no_bumper_right_no_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_Dock(self.actionPublisher))
        return self

    def no_bumper_right_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_Dock(self.actionPublisher))
        return self

    def no_bumper_pass_no_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_Dock(self.actionPublisher))
        return self

    def no_bumper_pass_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_Dock(self.actionPublisher))
        return self

    def no_bumper_dock_no_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_Dock(self.actionPublisher))
        return self

    def no_bumper_dock_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Should_Dock(self.actionPublisher))
        return self

    def bumper_none(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT)
        self.count += 1
        return self

    def bumper_left(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT)
        self.count += 1
        return self

    def bumper_uturn(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT)
        self.count += 1
        return self

    def bumper_right(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT)
        self.count += 1
        return self

    def bumper_pass(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT)
        self.count += 1
        return self

    def bumper_dock(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT)
        self.count += 1
        return self


class Collision_Intersection(Operational):
    def __init__(self, actionPublisher, count=1):
        super().__init__(actionPublisher)
        self.stateType = StateType.COLLISION_INTERSECTION
        self.count = count

    def no_bumper_none_no_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Handle_Intersection(self.actionPublisher))
        return self

    def no_bumper_none_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Handle_Intersection(self.actionPublisher))
        return self

    def no_bumper_left_no_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Handle_Intersection(self.actionPublisher))
        return self

    def no_bumper_left_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Handle_Intersection(self.actionPublisher))
        return self

    def no_bumper_uturn_no_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR, Handle_Intersection(self.actionPublisher))
        return self

    def no_bumper_uturn_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR, Handle_Intersection(self.actionPublisher))
        return self

    def no_bumper_right_no_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Handle_Intersection(self.actionPublisher))
        return self

    def no_bumper_right_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Handle_Intersection(self.actionPublisher))
        return self

    def no_bumper_pass_no_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Handle_Intersection(self.actionPublisher))
        return self

    def no_bumper_pass_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Handle_Intersection(self.actionPublisher))
        return self

    def no_bumper_dock_no_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Handle_Intersection(self.actionPublisher))
        return self

    def no_bumper_dock_wall(self):
        self.setLongAction(generateAction(Action.R_TURN.value), RIGHT_TURN_LIMIT * self.count * COLLISION_FACTOR,
                           Handle_Intersection(self.actionPublisher))
        return self

    def bumper_none(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT)
        self.count += 1
        return self

    def bumper_left(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT)
        self.count += 1
        return self

    def bumper_uturn(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT)
        self.count += 1
        return self

    def bumper_right(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT)
        self.count += 1
        return self

    def bumper_pass(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT)
        self.count += 1
        return self

    def bumper_dock(self):
        self.setLongAction(generateAction(Action.L_TURN.value), LEFT_TURN_LIMIT)
        self.count += 1
        return self


def generateAction(command, data=""):
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
