# This Python script requires a network connection and ssl certs before running.
# To check this automatically, use the fwdip_helper.sh script to run.

from pyrebase.pyrebase import Database
from pyrebase import initialize_app as initApp
import urllib.request as ur
from subprocess import check_output
from time import sleep
import os

# Fill out the config with the information from your Firebase Real time database
CONFIG = {
        "apiKey": "AIzaSyAWtAoHIuIGsiMT035-A6SyvvbeDhv6b0o",
        "authDomain": "create3-ip.firebaseapp.com",
        "databaseURL": "https://create3-ip-default-rtdb.firebaseio.com/",
        "storageBucket": "create3-ip.appspot.com"
}

# Set the CHILD to whatever you want to name the ip address entry
CHILD: str = "CREATE3-1"

def ip_post():

    ip_address = os.popen("ip route get 8.8.8.8 | awk '{gsub(\".*src\", \"\"); print $1; exit}'").read()
    print("IP address:", ip_address)

    firebase = initApp(CONFIG)
    db: Database = firebase.database()
    db.child(CHILD).set(ip_address)


if __name__ == "__main__":
	ip_post()
