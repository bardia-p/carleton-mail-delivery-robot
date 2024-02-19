from control import state_machine as state_machine

T1 = (False, "NAV_NONE", "-1:-1")
T2 = (False, "NAV_NONE", "10:0")
T3 = (False, "NAV_LEFT", "-1:-1")
T4 = (False, "NAV_LEFT", "10:0")
T5 = (False, "NAV_RIGHT", "-1:-1")
T6 = (False, "NAV_RIGHT", "10:0")
T7 = (False, "NAV_PASS", "-1:-1")
T8 = (False, "NAV_PASS", "10:0")
T9 = (False, "NAV_U_TURN", "-1:-1")
T10 = (False, "NAV_U_TURN", "10:0")
T11 = (False, "NAV_DOCK", "-1:-1")
T12 = (False, "NAV_DOCK", "10:0")
T13 = (True, "NAV_NONE", "10:0")
T14 = (True, "NAV_LEFT", "10:0")
T15 = (True, "NAV_RIGHT", "10:0")
T16 = (True, "NAV_PASS", "10:0")
T17 = (True, "NAV_U_TURN", "10:0")
T18 = (True, "NAV_DOCK", "10:0")

class ActionPublisherStub():
    '''
    The stub for the Action Publisher in the state machine
    There is nothing actually being published, it just stores a list of completed actions
    '''

    def __init__(self):
        self.actions = []
        self.action_data = []

    def publish(self, action):
        self.actions.append(action)

    
    def extract_data(self):
        '''
        Puts all the friendly names for completed actions into a list for easy comparison
        '''
        for action in self.actions:
            self.action_data.append(action.data.split(":")[0])
    
def send_update(state, update):
    newState= state.handleUpdate(update[0], update[1], update[2])
    if newState.isBusy:
        #newState.handleUpdate(update[0], update[1], update[2])S
        newState.longActionCount = newState.longActionLimit
        newState = newState.handleUpdate(update[0], update[1], update[2])
    return newState


def test_path_one():
    '''
    Test Path 1: Asserts that state transitions are good for right turns and that the completed actions
    are as expected
    '''

    actionPublisher = ActionPublisherStub()
    state = state_machine.No_Dest(actionPublisher)

    state = send_update(state, T2)
    assert state.stateType.value == "NO_DEST"
    state = send_update(state, T13)
    assert state.stateType.value == "COLLISION_NO_DEST"
    state = send_update(state, T15)
    assert state.stateType.value == "COLLISION_TURN_RIGHT"
    state = send_update(state, T6)
    assert state.stateType.value == "SHOULD_TURN_RIGHT"
    state = send_update(state, T1)
    assert state.stateType.value == "HANDLE_INTERSECTION"

    actionPublisher.extract_data()
    expected_actions = ["WALL_FOLLOW", "L_TURN", "L_TURN", "R_TURN", "R_TURN"]
    assert actionPublisher.action_data == expected_actions


def test_path_two():
    '''
    Test Path 2: Asserts that state transitions are good for left turns and that the completed actions
    are as expected
    '''

    actionPublisher = ActionPublisherStub()
    state = state_machine.No_Dest(actionPublisher)

    state = send_update(state, T2)
    assert state.stateType.value == "NO_DEST"
    state = send_update(state, T14)
    assert state.stateType.value == "COLLISION_TURN_LEFT"
    state = send_update(state, T4)
    assert state.stateType.value == "SHOULD_TURN_LEFT"
    state = send_update(state, T3)
    assert state.stateType.value == "HANDLE_INTERSECTION"
    state = send_update(state, T14)
    assert state.stateType.value == "COLLISION_INTERSECTION"
    state = send_update(state, T3)
    assert state.stateType.value == "HANDLE_INTERSECTION"

    actionPublisher.extract_data()
    expected_actions = ["WALL_FOLLOW", "L_TURN", "R_TURN", "L_TURN", "L_TURN", "R_TURN"]
    assert actionPublisher.action_data == expected_actions


def test_path_three():
    '''
    Test Path 3: Asserts that state transitions are good for passing and that the completed actions
    are as expected
    '''

    actionPublisher = ActionPublisherStub()
    state = state_machine.No_Dest(actionPublisher)

    state = send_update(state, T2)
    assert state.stateType.value == "NO_DEST"
    state = send_update(state, T16)
    assert state.stateType.value == "COLLISION_PASS"
    state = send_update(state, T8) 
    assert state.stateType.value == "SHOULD_PASS"
    state = send_update(state, T7)
    assert state.stateType.value == "HANDLE_INTERSECTION"

    actionPublisher.extract_data()
    expected_actions = ["WALL_FOLLOW", "L_TURN", "R_TURN", "FORWARD"]
    assert actionPublisher.action_data == expected_actions

def test_path_four():
    '''
    Test Path 4: Asserts that state transitions are good for docking and that the completed actions
    are as expected
    '''

    actionPublisher = ActionPublisherStub()
    state = state_machine.No_Dest(actionPublisher)

    state = send_update(state, T2)
    assert state.stateType.value == "NO_DEST"
    state = send_update(state, T18)
    assert state.stateType.value == "COLLISION_DOCK"
    state = send_update(state, T12)
    assert state.stateType.value == "SHOULD_DOCK"
    state = send_update(state, T11)
    assert state.stateType.value == "DOCKED"

    actionPublisher.extract_data()
    expected_actions = ["WALL_FOLLOW", "L_TURN", "R_TURN", "DOCK"]
    assert actionPublisher.action_data == expected_actions


def test_path_five():
    '''
    Test Path 5: Asserts that state the state transition into NOT_OPERATIONAL is good
    '''
    actionPublisher = ActionPublisherStub()
    state = state_machine.No_Dest(actionPublisher)

    #Path 5
    state = state_machine.No_Dest(actionPublisher)
    state = state.gotError()
    assert state.stateType.value == "NOT_OPERATIONAL"

def test_path_six():
    '''
    Test Path 6: 
    '''
    actionPublisher = ActionPublisherStub()
    state = state_machine.No_Dest(actionPublisher)
    state = send_update(state, T2)
    assert state.stateType.value == "NO_DEST"
    state = send_update(state, T15)
    assert state.stateType.value == "COLLISION_TURN_RIGHT"
    state = send_update(state, T13)
    assert state.stateType.value == "COLLISION_TURN_RIGHT"
    state = send_update(state, T14)
    assert state.stateType.value == "COLLISION_TURN_RIGHT"
    state = send_update(state, T15)
    assert state.stateType.value == "COLLISION_TURN_RIGHT"
    state = send_update(state, T16)
    assert state.stateType.value == "COLLISION_TURN_RIGHT"
    state = send_update(state, T18)
    assert state.stateType.value == "COLLISION_TURN_RIGHT"
    state = send_update(state, T1)
    assert state.stateType.value == "SHOULD_TURN_RIGHT"
    state = send_update(state, T2)
    assert state.stateType.value == "SHOULD_TURN_RIGHT"
    state = send_update(state, T4)
    assert state.stateType.value == "SHOULD_TURN_RIGHT"
    state = send_update(state, T6)
    assert state.stateType.value == "SHOULD_TURN_RIGHT"
    state = send_update(state, T8)
    assert state.stateType.value == "SHOULD_TURN_RIGHT"
    state = send_update(state, T12)
    assert state.stateType.value == "SHOULD_TURN_RIGHT"
    state = send_update(state, T3)
    assert state.stateType.value == "HANDLE_INTERSECTION"
    state = send_update(state, T1)
    assert state.stateType.value == "HANDLE_INTERSECTION"
    state = send_update(state, T3)
    assert state.stateType.value == "HANDLE_INTERSECTION"
    state = send_update(state, T5)
    assert state.stateType.value == "HANDLE_INTERSECTION"
    state = send_update(state, T7)
    assert state.stateType.value == "HANDLE_INTERSECTION"
    state = send_update(state, T11)
    assert state.stateType.value == "HANDLE_INTERSECTION"
    state = send_update(state, T13)
    assert state.stateType.value == "COLLISION_INTERSECTION"
    state = send_update(state, T13)
    assert state.stateType.value == "COLLISION_INTERSECTION"
    state = send_update(state, T14)
    assert state.stateType.value == "COLLISION_INTERSECTION"
    state = send_update(state, T15)
    assert state.stateType.value == "COLLISION_INTERSECTION"
    state = send_update(state, T16)
    assert state.stateType.value == "COLLISION_INTERSECTION"
    state = send_update(state, T18)
    assert state.stateType.value == "COLLISION_INTERSECTION"
    state = send_update(state, T1)
    assert state.stateType.value == "HANDLE_INTERSECTION"
    state = send_update(state, T15)
    assert state.stateType.value == "COLLISION_INTERSECTION"
    state = send_update(state, T2)
    assert state.stateType.value == "HANDLE_INTERSECTION"
    state = send_update(state, T16)
    assert state.stateType.value == "COLLISION_INTERSECTION"
    state = send_update(state, T4)
    assert state.stateType.value == "HANDLE_INTERSECTION"
    state = send_update(state, T18)
    assert state.stateType.value == "COLLISION_INTERSECTION"
    state = send_update(state, T5)
    assert state.stateType.value == "HANDLE_INTERSECTION"
    state = send_update(state, T18)
    assert state.stateType.value == "COLLISION_INTERSECTION"
    state = send_update(state, T6)
    assert state.stateType.value == "HANDLE_INTERSECTION"
    state = send_update(state, T18)
    assert state.stateType.value == "COLLISION_INTERSECTION"
    state = send_update(state, T7)
    assert state.stateType.value == "HANDLE_INTERSECTION"
    state = send_update(state, T18)
    assert state.stateType.value == "COLLISION_INTERSECTION"
    state = send_update(state, T8)
    assert state.stateType.value == "HANDLE_INTERSECTION"
    state = send_update(state, T18)
    assert state.stateType.value == "COLLISION_INTERSECTION"
    state = send_update(state, T11)
    assert state.stateType.value == "HANDLE_INTERSECTION"
    state = send_update(state, T18)
    assert state.stateType.value == "COLLISION_INTERSECTION"
    state = send_update(state, T12)
    assert state.stateType.value == "HANDLE_INTERSECTION"
    state = send_update(state, T2)
    assert state.stateType.value == "NO_DEST"
    state = send_update(state, T1)
    assert state.stateType.value == "NO_DEST"
    state = send_update(state, T2)
    assert state.stateType.value == "NO_DEST"
    state = send_update(state, T13)
    assert state.stateType.value == "COLLISION_NO_DEST"
    state = send_update(state, T13)
    assert state.stateType.value == "COLLISION_NO_DEST"
    state = send_update(state, T1)
    assert state.stateType.value == "NO_DEST"
    state = send_update(state, T13)
    assert state.stateType.value == "COLLISION_NO_DEST"
    state = send_update(state, T2)
    assert state.stateType.value == "NO_DEST"
    state = send_update(state, T15)
    assert state.stateType.value == "COLLISION_TURN_RIGHT"
    state = send_update(state, T2)
    assert state.stateType.value == "SHOULD_TURN_RIGHT"
    state = send_update(state, T13)
    assert state.stateType.value == "COLLISION_TURN_RIGHT"
    state = send_update(state, T3)
    assert state.stateType.value == "SHOULD_TURN_RIGHT"
    state = send_update(state, T14)
    assert state.stateType.value == "COLLISION_TURN_RIGHT"
    state = send_update(state, T4)
    assert state.stateType.value == "SHOULD_TURN_RIGHT"
    state = send_update(state, T15)
    assert state.stateType.value == "COLLISION_TURN_RIGHT"
    state = send_update(state, T5)
    assert state.stateType.value == "SHOULD_TURN_RIGHT"
    state = send_update(state, T16)
    assert state.stateType.value == "COLLISION_TURN_RIGHT"
    state = send_update(state, T7)
    assert state.stateType.value == "SHOULD_TURN_RIGHT"
    state = send_update(state, T18)
    assert state.stateType.value == "COLLISION_TURN_RIGHT"
    state = send_update(state, T8)
    assert state.stateType.value == "SHOULD_TURN_RIGHT"
    state = send_update(state, T18)
    assert state.stateType.value == "COLLISION_TURN_RIGHT"
    state = send_update(state, T11)
    assert state.stateType.value == "SHOULD_TURN_RIGHT"
    state = send_update(state, T18)
    assert state.stateType.value == "COLLISION_TURN_RIGHT"
    state = send_update(state, T12)
    assert state.stateType.value == "SHOULD_TURN_RIGHT"
    
def test_path_seven():
    '''
    Test path 7:
    '''
    actionPublisher = ActionPublisherStub()
    state = state_machine.No_Dest(actionPublisher)
    state = send_update(state, T2)
    assert state.stateType.value == "NO_DEST"
    state = send_update(state, T14)
    assert state.stateType.value == "COLLISION_TURN_LEFT"
    state = send_update(state, T13)
    assert state.stateType.value == "COLLISION_TURN_LEFT"
    state = send_update(state, T14)
    assert state.stateType.value == "COLLISION_TURN_LEFT"
    state = send_update(state, T15)
    assert state.stateType.value == "COLLISION_TURN_LEFT"
    state = send_update(state, T16)
    assert state.stateType.value == "COLLISION_TURN_LEFT"
    state = send_update(state, T18)
    assert state.stateType.value == "COLLISION_TURN_LEFT"
    state = send_update(state, T1)
    assert state.stateType.value == "SHOULD_TURN_LEFT"
    state = send_update(state, T2)
    assert state.stateType.value == "SHOULD_TURN_LEFT"
    state = send_update(state, T4)
    assert state.stateType.value == "SHOULD_TURN_LEFT"
    state = send_update(state, T6)
    assert state.stateType.value == "SHOULD_TURN_LEFT"
    state = send_update(state, T8)
    assert state.stateType.value == "SHOULD_TURN_LEFT"
    state = send_update(state, T12)
    assert state.stateType.value == "SHOULD_TURN_LEFT"
    state = send_update(state, T1)
    assert state.stateType.value == "HANDLE_INTERSECTION"
    state = send_update(state, T4)
    assert state.stateType.value == "NO_DEST"
    state = send_update(state, T14)
    assert state.stateType.value == "COLLISION_TURN_LEFT"
    state = send_update(state, T2)
    assert state.stateType.value == "SHOULD_TURN_LEFT"
    state = send_update(state, T5)
    assert state.stateType.value == "HANDLE_INTERSECTION"
    state = send_update(state, T6)
    assert state.stateType.value == "NO_DEST"
    state = send_update(state, T14)
    assert state.stateType.value == "COLLISION_TURN_LEFT"
    state = send_update(state, T4)
    assert state.stateType.value == "SHOULD_TURN_LEFT"
    state = send_update(state, T7)
    assert state.stateType.value == "HANDLE_INTERSECTION"
    state = send_update(state, T8)
    assert state.stateType.value == "NO_DEST"
    state = send_update(state, T14)
    assert state.stateType.value == "COLLISION_TURN_LEFT"
    state = send_update(state, T5)
    assert state.stateType.value == "SHOULD_TURN_LEFT"
    state = send_update(state, T11)
    assert state.stateType.value == "HANDLE_INTERSECTION"
    state = send_update(state, T12)
    assert state.stateType.value == "NO_DEST"
    state = send_update(state, T14)
    assert state.stateType.value == "COLLISION_TURN_LEFT"
    state = send_update(state, T6)
    assert state.stateType.value == "SHOULD_TURN_LEFT"
    state = send_update(state, T13)
    assert state.stateType.value == "COLLISION_TURN_LEFT"
    state = send_update(state, T7)
    assert state.stateType.value == "SHOULD_TURN_LEFT"
    state = send_update(state, T14)
    assert state.stateType.value == "COLLISION_TURN_LEFT"
    state = send_update(state, T8)
    assert state.stateType.value == "SHOULD_TURN_LEFT"
    state = send_update(state, T15)
    assert state.stateType.value == "COLLISION_TURN_LEFT"
    state = send_update(state, T11)
    assert state.stateType.value == "SHOULD_TURN_LEFT"
    state = send_update(state, T16)
    assert state.stateType.value == "COLLISION_TURN_LEFT"
    state = send_update(state, T12)
    assert state.stateType.value == "SHOULD_TURN_LEFT"
    state = send_update(state, T18)
    assert state.stateType.value == "COLLISION_TURN_LEFT"

def test_path_eight():
    '''
    Test path 8:
    '''
    actionPublisher = ActionPublisherStub()
    state = state_machine.No_Dest(actionPublisher)
    state = send_update(state, T2)
    assert state.stateType.value == "NO_DEST"
    state = send_update(state, T13)
    assert state.stateType.value == "COLLISION_NO_DEST"
    state = send_update(state, T16)
    assert state.stateType.value == "COLLISION_PASS"
    state = send_update(state, T13)
    assert state.stateType.value == "COLLISION_PASS"
    state = send_update(state, T14)
    assert state.stateType.value == "COLLISION_PASS"
    state = send_update(state, T15)
    assert state.stateType.value == "COLLISION_PASS"
    state = send_update(state, T16)
    assert state.stateType.value == "COLLISION_PASS"
    state = send_update(state, T18)
    assert state.stateType.value == "COLLISION_PASS"
    state = send_update(state, T1)
    assert state.stateType.value == "SHOULD_PASS"
    state = send_update(state, T13)
    assert state.stateType.value == "COLLISION_PASS"
    state = send_update(state, T2)
    assert state.stateType.value == "SHOULD_PASS"
    state = send_update(state, T14)
    assert state.stateType.value == "COLLISION_PASS"
    state = send_update(state, T3)
    assert state.stateType.value == "SHOULD_PASS"
    state = send_update(state, T15)
    assert state.stateType.value == "COLLISION_PASS"
    state = send_update(state, T4)
    assert state.stateType.value == "SHOULD_PASS"
    state = send_update(state, T16)
    assert state.stateType.value == "COLLISION_PASS"
    state = send_update(state, T5)
    assert state.stateType.value == "SHOULD_PASS"
    state = send_update(state, T18)
    assert state.stateType.value == "COLLISION_PASS"
    state = send_update(state, T6)
    assert state.stateType.value == "SHOULD_PASS"
    state = send_update(state, T18)
    assert state.stateType.value == "COLLISION_PASS"
    state = send_update(state, T7)
    assert state.stateType.value == "SHOULD_PASS"
    state = send_update(state, T18)
    assert state.stateType.value == "COLLISION_PASS"
    state = send_update(state, T11)
    assert state.stateType.value == "SHOULD_PASS"
    state = send_update(state, T18)
    assert state.stateType.value == "COLLISION_PASS"
    state = send_update(state, T12)
    assert state.stateType.value == "SHOULD_PASS"
    state = send_update(state, T2)
    assert state.stateType.value == "SHOULD_PASS"
    state = send_update(state, T4)
    assert state.stateType.value == "SHOULD_PASS"
    state = send_update(state, T6)
    assert state.stateType.value == "SHOULD_PASS"
    state = send_update(state, T8)
    assert state.stateType.value == "SHOULD_PASS"
    state = send_update(state, T12)
    assert state.stateType.value == "SHOULD_PASS"
    state = send_update(state, T1)
    assert state.stateType.value == "HANDLE_INTERSECTION"
    state = send_update(state, T2)
    assert state.stateType.value == "NO_DEST"
    state = send_update(state, T13)
    assert state.stateType.value == "COLLISION_NO_DEST"
    state = send_update(state, T7)
    assert state.stateType.value == "SHOULD_PASS"
    state = send_update(state, T3)
    assert state.stateType.value == "HANDLE_INTERSECTION"
    state = send_update(state, T2)
    assert state.stateType.value == "NO_DEST"
    state = send_update(state, T13)
    assert state.stateType.value == "COLLISION_NO_DEST"
    state = send_update(state, T8)
    assert state.stateType.value == "SHOULD_PASS"
    state = send_update(state, T5)
    assert state.stateType.value == "HANDLE_INTERSECTION"
    state = send_update(state, T2)
    assert state.stateType.value == "NO_DEST"
    state = send_update(state, T13)
    assert state.stateType.value == "COLLISION_NO_DEST"
    state = send_update(state, T7)
    assert state.stateType.value == "SHOULD_PASS"
    state = send_update(state, T11)
    assert state.stateType.value == "HANDLE_INTERSECTION"


def test_path_nine():
    '''
    Test path 9:
    '''
    actionPublisher = ActionPublisherStub()
    state = state_machine.No_Dest(actionPublisher)
    state = send_update(state, T2)
    assert state.stateType.value == "NO_DEST"
    state = send_update(state, T13)
    assert state.stateType.value == "COLLISION_NO_DEST"
    state = send_update(state, T11)
    assert state.stateType.value == "SHOULD_DOCK"
    state = send_update(state, T2)
    assert state.stateType.value == "SHOULD_DOCK"
    state = send_update(state, T4)
    assert state.stateType.value == "SHOULD_DOCK"
    state = send_update(state, T6)
    assert state.stateType.value == "SHOULD_DOCK"
    state = send_update(state, T8)
    assert state.stateType.value == "SHOULD_DOCK"
    state = send_update(state, T12)
    assert state.stateType.value == "SHOULD_DOCK"
    state = send_update(state, T13)
    assert state.stateType.value == "COLLISION_DOCK"
    state = send_update(state, T13)
    assert state.stateType.value == "COLLISION_DOCK"
    state = send_update(state, T14)
    assert state.stateType.value == "COLLISION_DOCK"
    state = send_update(state, T15)
    assert state.stateType.value == "COLLISION_DOCK"
    state = send_update(state, T16)
    assert state.stateType.value == "COLLISION_DOCK"
    state = send_update(state, T18)
    assert state.stateType.value == "COLLISION_DOCK"
    state = send_update(state, T1)
    assert state.stateType.value == "SHOULD_DOCK"
    state = send_update(state, T14)
    assert state.stateType.value == "COLLISION_DOCK"
    state = send_update(state, T2)
    assert state.stateType.value == "SHOULD_DOCK"
    state = send_update(state, T15)
    assert state.stateType.value == "COLLISION_DOCK"
    state = send_update(state, T3)
    assert state.stateType.value == "SHOULD_DOCK"
    state = send_update(state, T16)
    assert state.stateType.value == "COLLISION_DOCK"
    state = send_update(state, T4)
    assert state.stateType.value == "SHOULD_DOCK"
    state = send_update(state, T18)
    assert state.stateType.value == "COLLISION_DOCK"
    state = send_update(state, T5)
    assert state.stateType.value == "SHOULD_DOCK"
    state = send_update(state, T18)
    assert state.stateType.value == "COLLISION_DOCK"
    state = send_update(state, T6)
    assert state.stateType.value == "SHOULD_DOCK"
    state = send_update(state, T18)
    assert state.stateType.value == "COLLISION_DOCK"
    state = send_update(state, T7)
    assert state.stateType.value == "SHOULD_DOCK"
    state = send_update(state, T18)
    assert state.stateType.value == "COLLISION_DOCK"
    state = send_update(state, T8)
    assert state.stateType.value == "SHOULD_DOCK"
    state = send_update(state, T18)
    assert state.stateType.value == "COLLISION_DOCK"
    state = send_update(state, T11)
    assert state.stateType.value == "SHOULD_DOCK"
    state = send_update(state, T1)
    assert state.stateType.value == "DOCKED"

def test_path_ten():
    '''
    Test path 10:
    '''
    actionPublisher = ActionPublisherStub()
    state = state_machine.No_Dest(actionPublisher)
    state = send_update(state, T2)
    assert state.stateType.value == "NO_DEST"
    state = send_update(state, T11)
    assert state.stateType.value == "SHOULD_DOCK"
    state = send_update(state, T3)
    assert state.stateType.value == "DOCKED"

def test_path_eleven():
    '''
    Test path 11:
    '''
    actionPublisher = ActionPublisherStub()
    state = state_machine.No_Dest(actionPublisher)
    state = send_update(state, T2)
    assert state.stateType.value == "NO_DEST"
    state = send_update(state, T12)
    assert state.stateType.value == "SHOULD_DOCK"
    state = send_update(state, T5)
    assert state.stateType.value == "DOCKED"
    
def test_path_twelve():
    '''
    Test path 12:
    '''
    actionPublisher = ActionPublisherStub()
    state = state_machine.No_Dest(actionPublisher)
    state = send_update(state, T2)
    assert state.stateType.value == "NO_DEST"
    state = send_update(state, T13)
    assert state.stateType.value == "COLLISION_NO_DEST"
    state = send_update(state, T18)
    assert state.stateType.value == "COLLISION_DOCK"
    state = send_update(state, T1)
    assert state.stateType.value == "SHOULD_DOCK"
    state = send_update(state, T7)
    assert state.stateType.value == "DOCKED"
    
def test_path_thirteen():
    '''
    Test path 13:
    '''
    actionPublisher = ActionPublisherStub()
    state = state_machine.No_Dest(actionPublisher)
    state = send_update(state, T2)
    assert state.stateType.value == "NO_DEST"
    state = send_update(state, T7)
    assert state.stateType.value == "SHOULD_PASS"
    state = send_update(state, T7)
    assert state.stateType.value == "HANDLE_INTERSECTION"
    state = send_update(state, T8)
    assert state.stateType.value == "NO_DEST"
    state = send_update(state, T8)
    assert state.stateType.value == "SHOULD_PASS"
    state = send_update(state, T7)
    assert state.stateType.value == "HANDLE_INTERSECTION"
    state = send_update(state, T8)
    assert state.stateType.value == "NO_DEST"
    state = send_update(state, T13)
    assert state.stateType.value == "COLLISION_NO_DEST"
    state = send_update(state, T5)
    assert state.stateType.value == "SHOULD_TURN_RIGHT"
    state = send_update(state, T5)
    assert state.stateType.value == "HANDLE_INTERSECTION"
    state = send_update(state, T8)
    assert state.stateType.value == "NO_DEST"
    state = send_update(state, T13)
    assert state.stateType.value == "COLLISION_NO_DEST"
    state = send_update(state, T6)
    assert state.stateType.value == "SHOULD_TURN_RIGHT"
    state = send_update(state, T7)
    assert state.stateType.value == "HANDLE_INTERSECTION"
    state = send_update(state, T8)
    assert state.stateType.value == "NO_DEST"
    state = send_update(state, T13)
    assert state.stateType.value == "COLLISION_NO_DEST"
    state = send_update(state, T6)
    assert state.stateType.value == "SHOULD_TURN_RIGHT"
    state = send_update(state, T11)
    assert state.stateType.value == "HANDLE_INTERSECTION"
    state = send_update(state, T8)
    assert state.stateType.value == "NO_DEST"
    state = send_update(state, T13)
    assert state.stateType.value == "COLLISION_NO_DEST"
    state = send_update(state, T3)
    assert state.stateType.value == "SHOULD_TURN_LEFT"
    state = send_update(state, T11)
    assert state.stateType.value == "HANDLE_INTERSECTION"
    state = send_update(state, T8)
    assert state.stateType.value == "NO_DEST"
    state = send_update(state, T13)
    assert state.stateType.value == "COLLISION_NO_DEST"
    state = send_update(state, T4)
    assert state.stateType.value == "SHOULD_TURN_LEFT"
    state = send_update(state, T11)
    assert state.stateType.value == "HANDLE_INTERSECTION"
    state = send_update(state, T8)
    assert state.stateType.value == "NO_DEST"
    state = send_update(state, T5)
    assert state.stateType.value == "SHOULD_TURN_RIGHT"
    state = send_update(state, T11)
    assert state.stateType.value == "HANDLE_INTERSECTION"
    state = send_update(state, T8)
    assert state.stateType.value == "NO_DEST"
    state = send_update(state, T6)
    assert state.stateType.value == "SHOULD_TURN_RIGHT"
    state = send_update(state, T11)
    assert state.stateType.value == "HANDLE_INTERSECTION"
    state = send_update(state, T8)
    assert state.stateType.value == "NO_DEST"
    state = send_update(state, T4)
    assert state.stateType.value == "SHOULD_TURN_LEFT"
    state = send_update(state, T11)
    assert state.stateType.value == "HANDLE_INTERSECTION"
    state = send_update(state, T8)
    assert state.stateType.value == "NO_DEST"
    state = send_update(state, T3)
    assert state.stateType.value == "SHOULD_TURN_LEFT"


