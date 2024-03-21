# Firebase IP Service

This tool can be used to run a systemd service that posts the IP of a raspberry pi to a Firebase Realtime Database

## Firebase Setup and Script Configuration

The following steps should be followed to create a Firebase database and configure the script.

1. Go to https://console.firebase.google.com/ and login to a Google account (can make a new one or use an existing account)
2. Click on the “add project” button and give the project a name (ex. create-ip)
3. Create the project
4. Add a realtime database and anonymous authentication to the project
5. Find the API key from the project settings page
6. Edit the fwdip.py file and add your Firebase information to the CONFIG dictionary. The apiKey value should be your project's API key, the authDomain value should be "<PROJECT_NAME>.firebaseapp.com", the database URL value should be "https://<PROJECT_NAME>-default-rtdb.firebaseio.com/", and storageBucket should be "<PROJECT_NAME>.appspot.com". Replace <PROJECT_NAME> values with the project name chosen in step 2 (ex. "authDomain: "create3-ip.firebaseapp.com").
7. Configure the "CHILD" value with the name you want sent to the firebase. When using mutliple create 3 robots, change this value so that you know the associated IP of each robot.

## Installing Pyrebase

In order to post the IP of the pi to the firebase database, the Pyrebase Python library must be installed.

You can install Pyrebase by running the following command:

```
pip3 install Pyrebase4
```

#####  Note: Pyrebase4 is recomended as it solves issues with the requests library in Python 3.10

## Running the service

Before setting up the service to run, modify the ExecStart path in the fwdip.service file. The ExecStart path should point to the fwdip_helper.sh script which must be in the same folder as the fwdip.py script. You may also edit the log file paths as desired.

Now to have the service run on system boot, copy the fwdip.service file to the /etc/systemd/system folder and start it by running the following commands.

```
cp fwdip.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl start fwdip.service
```

The status of the service can then be check as follows:

```
sudo systemctl status fwdip.service
```

## Using the service

With the above configuration completed, the Raspberry Pi will post its IP address to your Firebase Realtime Database every time it boots. To access the IP, navigate to your Firebase project page and click on the "realtime database" button.