from tools.nav_parser import loadMap

class Map:

    '''
    Defines the hashmap that the robot uses for its routes.
    Remember: the map has a hashmap within a hashmap as values. Therefore, you would retrieve a value from within the map as follows:

    routing_map["Loeb1"]["Southam"] -> returns U-TURN
    '''

    def __init__(self):
        '''
        Constructor for the map. Essentially initializes the map into a hashmap.
        '''
        self.routing_map = loadMap()

    def getMap(self):
        '''
        Typical getter for the whole routing map.
        '''
        return self.routing_map

    def getValue(self, key):
        '''
        General method to return the value from a given key in a key-value pair.
        '''
        return self.routing_map.get(key)
