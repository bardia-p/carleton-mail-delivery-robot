# Beacon Analyzer

The goal of this tool is to allow users to analyze a beacon from different distances.

## Installing bluepy

To use the application, please make sure that you have bluepy installed.

You can install bluepy through the following command:

```
sudo apt install bluepy
```

## Running the application

You can run the application by simply running the following command:

```
python3 beacon_analyzer.py <beacon>
```

## Using the application

### Running the experiment
1. Place the Raspberry Pi in a static location on one end of a hallway.
2. Run the application to collect rolling average for the RSSI of a target beacon.
3. Place the beacon at several known distances away from the Raspberry Pi
4. Record rolling average measurement with the beacon both by placing the beacon at the same level as the robot and as close to the ceiling as possible at known heights.

    *Make sure to pause between measurements for the rolling average to settle.*

5. Repeat the experiment for a second beacon.

### Analysis
- Compare the RSSI values at known distances while paying close attention to height differences.

### Example Results

**1m height difference, next to metal window**

```
Beacon 1
Distance, floor time, floor rssi, ceiling time, ceiling rssi
1m,12:01:53,-43,12:02:36,-60
2.5m,12:03:31,-58,12:04:30,-59
4m,12:05:35,-65,12:06:35,-70
``````

**Middle of hallway with no height differences**

```
Beacon 1 (c2a8)
Distance, floor time, floor rssi
1m,12:15:26,-58
2.5m,12:16:21,-65
4m,12:17:07,-70
5.5m,12:18:38,-67
7m,12:20:22,-71
8.5m,12:22:11,-75
10m,12:24:09,-68
11.5m,12:26:27,-71

Beacon 2 (1ed7)
Distance, floor time, floor rssi
1m,12:31:16,-55
2.5m,12:32:02,-67
4m,12:34:00,-68
5.5m,12:35:27,-77
7m,12:36:53,-74
8.5m,12:37:45,-75
10m,12:38:34,-72
11.5m,12:39:31,-80
```