from control import state_machine as state_machine

class ActionPublisherStub():
    def __init__(self):
        self.actions = []

    def publish(self, action):
        self.actions.append(action)


def test_all_states():
    actionPublisher = ActionPublisherStub()
    state = state_machine.No_Dest(actionPublisher)

    # Path 1
    state = state.handleUpdate(False, "NAV_NONE", "10:0")
    assert state.stateType.value == "NO_DEST"
    state = state.handleUpdate(True, "NAV_NONE", "10:-0")
    assert state.stateType.value == "COLLISION_NO_DEST"
    state = state.handleUpdate(True, "NAV_RIGHT", "10:0")
    assert state.stateType.value == "COLLISION_TURN_RIGHT"
    state = state.handleUpdate(False, "NAV_RIGHT", "10:0")
    assert state.stateType.value == "SHOULD_TURN_RIGHT"
    state = state.handleUpdate(False, "NAV_NONE", "-1:-1")
    assert state.stateType.value == "HANDLE_INTERSECTION"

    #Path 2
    state = state.handleUpdate(False, "NAV_NONE", "10:0")
    assert state.stateType.value == "NO_DEST"
    state = state.handleUpdate(True, "NAV_LEFT", "10:0")
    assert state.stateType.value == "COLLISION_TURN_LEFT"
    state = state.handleUpdate(False, "NAV_LEFT", "10:0")
    assert state.stateType.value == "SHOULD_TURN_LEFT"
    state = state.handleUpdate(False, "NAV_LEFT", "-1:-1")
    assert state.stateType.value == "HANDLE_INTERSECTION"
    state = state.handleUpdate(True, "NAV_LEFT", "-1:-1")
    assert state.stateType.value == "COLLISION_INTERSECTION"
    state = state.handleUpdate(False, "NAV_LEFT", "-1:-1")
    assert state.stateType.value == "HANDLE_INTERSECTION"

    #Path 3
    state = state.handleUpdate(False, "NAV_NONE", "10:0")
    assert state.stateType.value == "NO_DEST"
    state = state.handleUpdate(True, "NAV_PASS", "10:0")
    assert state.stateType.value == "COLLISION_PASS"
    state = state.handleUpdate(False, "NAV_PASS", "10:0") #False with NAV_PASS but works with other values... not sure why
    assert state.stateType.value == "SHOULD_PASS", "THIS IS AN ERROR"
    state = state.handleUpdate(False, "NAV_PASS", "-1:-1")
    assert state.stateType.value == "HANDLE_INTERSECTION"

    #Path 4
    state = state.handleUpdate(False, "NAV_NONE", "10:0")
    assert state.stateType.value == "NO_DEST"
    state = state.handleUpdate(True, "NAV_DOCK", "10:0")
    assert state.stateType.value == "COLLISION_DOCK"
    state = state.handleUpdate(False, "NAV_DOCK", "10:0")
    assert state.stateType.value == "SHOULD_DOCK"
    state = state.handleUpdate(False, "NAV_DOCK", "-1:-1")
    assert state.stateType.value == "DOCKED"

    #Path 5
    state = state_machine.No_Dest(actionPublisher)
    state = state.gotError()
    assert state.stateType.value == "NOT_OPERATIONAL"