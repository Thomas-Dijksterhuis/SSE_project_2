# Interterrestrial Life Detection Hardware Guide

This guide explains the setup and operation of the Interterrestrial Life Detection hardware for audio detection. It covers the system architecture, status indicators, deep sleep functionality, and SD card configuration.

## System Logic

The system consists of three continuously running tasks.

### Main Task

The main task continuously monitors:

- SD card connectivity
- Wi-Fi connectivity

The status of these connections can be used for debugging and monitoring the operation of the other two tasks.

### Audio Processing Task

The audio processing task:

- Reads stereo UART audio data from the ADC connected to the ESP32.
- Samples audio at **20 kHz**.
- Places the captured audio data into a queue for further processing.

### Data Processing Task

The data processing task only runs when an SD card is connected.

The task has two operating modes.

#### Local Recording Mode (No Wi-Fi)

When no Wi-Fi connection is available:

1. Audio data is read from the queue.
2. The data is stored locally on the SD card.
3. Audio is saved in **20-second WAV files**.

#### Streaming Mode (Wi-Fi Connected)

When a Wi-Fi connection is available:

1. Audio data is read from the queue.
2. The data is transmitted to a receiver over UDP.
3. Audio is no longer stored as local recordings unless transmission fails.

---

## Setup

An SD card must be installed for the system to operate correctly.

### 1. Create the `.connected` File

The SD card must contain a file named:

```text
.connected
```

The file may be empty. The system only checks whether the file exists.

### 2. Create `Network.json`

The SD card must contain a file named:

```text
Network.json
```

This file contains the network configuration used by the device.

Example:

```json
{
    "SSID": "<WiFi_name>",
    "Password": "<WiFi_password>",
    "ReceiverIpAddress": "192.168.1.226",
    "DeviceId": "ESP32_1"
}
```

| Field | Description |
|---------|-------------|
| `SSID` | Name of the Wi-Fi network |
| `Password` | Wi-Fi password |
| `ReceiverIpAddress` | IP address of the UDP receiver |
| `DeviceId` | Unique identifier of the device |

### 3. Create `Schedule.json`

The SD card must contain a file named:

```text
Schedule.json
```

This file defines the operating schedule of the device.

Example:

```json
{
    "mode": "sampling",
    "schedule": [
        {
            "durationMinutes": 10,
            "periodMinutes": 20
        }
    ]
}
```

#### Schedule Modes

| Mode | Description |
|--------|-------------|
| `constant` | Continuous operation |
| `sampling` | Periodic operation according to the configured schedule |

#### Schedule Parameters

| Parameter | Description |
|------------|-------------|
| `durationMinutes` | Number of minutes the device records during each cycle |
| `periodMinutes` | Total duration of one cycle |

In the example above, the device records for **10 minutes every 20 minutes**.

---

## Stored Audio Files

The system automatically creates a folder named:

```text
missed_transmissions
```

This folder contains audio recordings that could not be transmitted because no Wi-Fi connection was available.

### Directory Structure

```text
SD Card
│
├── missed_transmissions
│   ├── no_date
│   │   ├── Audio_001.wav
│   │   ├── Audio_002.wav
│   │   └── ...
│   │
│   └── YYYY-MM-DD
│       ├── HH-MM-SS_HH-MM-SS.wav
│       └── ...
│
├── Schedule.json
├── Network.json
└── .connected
```

### Storage Behavior

#### No Date Available

If the device has never obtained a valid date and time (for example, because Wi-Fi was never available), recordings are stored in:

```text
missed_transmissions/no_date
```

Example:

```text
Audio_001.wav
Audio_002.wav
```

#### Date Available

Once the device has synchronized its date and time, recordings are stored in dated folders.

Example:

```text
missed_transmissions/
└── 2026-06-09/
    └── 14-30-00_14-30-20.wav
```

The filename indicates the start and end time of the recording.

---

## Deep Sleep Button

The device includes a dedicated deep sleep button.

When pressed, the ESP32 enters deep sleep mode to reduce power consumption. When the button is presssed again, the system wakes up again. This also works when the device went into sleep mode using the schedule.

---

## Status LED

The status LED provides a visual indication of the system state.

| LED State | LED Color | Meaning |
|-----------|-----------|---------|
| DEVICEMODE_OFF | Off|  Device is powered off or in deep sleep |
| DEVICEMODE_STARTING | Yellow| System is starting up |
| DEVICEMODE_NO_SD | Red | The system could not detect an SD card or the .connected file is missing |
| DEVICEMODE_SD_ERROR | Purple | The Schedule.json or Network.json is missing |
| DEVICEMODE_SD_FULL | White | The SD card is 95% full, clean it or put in another card |
| DEVICEMODE_SD | Blue | The system is running normally and it is recording to the local SD card. The recordings can be found in the folder called missed_transmissions |
| DEVICEMODE_WIFI | Green | The system is running normally and it is sending the recording to the receiver IP over WiFi |