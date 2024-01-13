import os
import csv

CONFIG_DIR = os.getcwd() + "/src/carleton-mail-delivery-robot/mail_delivery_robot/config/"
BEACONS_FILE = "beacons.csv"
CONFIG_FILE = "config.csv"

def parseCSVFile(filename, parse_numbers = False):
    '''
    Parses a CSV file located in te config directory and returns its contents as a dictionary.

    @param filename: The name of the csv file to parse.

    @return the contents of the parsed file.
    '''
    result = {}
    with open(CONFIG_DIR + filename) as csvfile:
        reader = csv.reader(csvfile, delimiter=",")
        for row in reader:
            result[row[0]] = row[1] if not parse_numbers else float(parse_numbers)
    return result

def loadBeacons():
    '''
    Loads all the beacons from the beacons file.
    '''
    return parseCSVFile(BEACONS_FILE)

def loadConfig():
    '''
    Loads the global config from the config file.
    '''
    return parseCSVFile(CONFIG_FILE, True)
