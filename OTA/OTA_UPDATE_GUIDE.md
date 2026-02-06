# Smart Glasses BLE OTA Update Guide

## Files You Need

You received 3 files:
1. **main.c** — Drop-in replacement for your existing main.c
2. **partitions_ota.csv** — New partition table with two OTA slots
3. **This guide** — Build instructions + nRF Connect steps

---

## Part 1: Project Changes Before Building

### Step 1: Copy files into your project

```
copy main.c  "C:\Users\dgrec\...\Code-Glasses\main\main.c"
copy partitions_ota.csv  "C:\Users\dgrec\...\Code-Glasses\partitions_ota.csv"
```

### Step 2: Change sdkconfig settings

Open `sdkconfig` in a text editor and make these changes:

**Change flash size from 2MB to 4MB** (your chip has 4MB):
```
Find:    CONFIG_ESPTOOLPY_FLASHSIZE_2MB=y
Replace: # CONFIG_ESPTOOLPY_FLASHSIZE_2MB is not set

Find:    # CONFIG_ESPTOOLPY_FLASHSIZE_4MB is not set
Replace: CONFIG_ESPTOOLPY_FLASHSIZE_4MB=y

Find:    CONFIG_ESPTOOLPY_FLASHSIZE="2MB"
Replace: CONFIG_ESPTOOLPY_FLASHSIZE="4MB"
```

**Change partition table to custom OTA layout**:
```
Find:    CONFIG_PARTITION_TABLE_SINGLE_APP=y
Replace: # CONFIG_PARTITION_TABLE_SINGLE_APP is not set

Find:    # CONFIG_PARTITION_TABLE_CUSTOM is not set
Replace: CONFIG_PARTITION_TABLE_CUSTOM=y

Find:    CONFIG_PARTITION_TABLE_CUSTOM_FILENAME="partitions.csv"
Replace: CONFIG_PARTITION_TABLE_CUSTOM_FILENAME="partitions_ota.csv"

Find:    CONFIG_PARTITION_TABLE_FILENAME="partitions_singleapp.csv"
Replace: CONFIG_PARTITION_TABLE_FILENAME="partitions_ota.csv"
```

Or alternatively, delete the `build` folder and `sdkconfig`, then run:
```
idf.py set-target esp32
idf.py menuconfig
```
And set these in the menu:
- Serial flasher config → Flash size → **4 MB**
- Partition Table → Partition Table → **Custom partition table CSV**
- Partition Table → Custom partition CSV file → **partitions_ota.csv**

### Step 3: Build

```
cd "C:\Users\dgrec\...\Code-Glasses"
idf.py fullclean
idf.py build
```

### Step 4: Flash (FIRST TIME — must use wired programmer)

Flash all three files using the Flash Download Tool:

| File | Address |
|------|---------|
| build\bootloader\bootloader.bin | 0x1000 |
| build\partition_table\partition-table.bin | 0x8000 |
| build\ESP32_Ble.bin | 0x10000 |

Settings: SPI SPEED 40MHz, SPI MODE DIO, DoNotChgBin checked.

**ERASE first**, then START.

After this initial flash, all future updates can go over Bluetooth.

---

## Part 2: OTA Update via nRF Connect

### What You Need
- **nRF Connect** app (free, iOS or Android)
- The new **ESP32_Ble.bin** file on your phone
  - Build new firmware on PC
  - Copy `build\ESP32_Ble.bin` to your phone (email, AirDrop, Google Drive, etc.)

### Step-by-Step in nRF Connect

#### 1. Connect to the Glasses

1. Open nRF Connect
2. Tap **SCAN**
3. Find **Smart_Glasses** in the device list
4. Tap **CONNECT**

#### 2. Discover Services

After connecting, you'll see the service list. Find:

```
Service: 0x00FF
  ├── Characteristic 0xFF01  (Control)    — Read, Write
  ├── Characteristic 0xFF02  (OTA Data)   — Write, Write Without Response
  └── Characteristic 0xFF03  (Status)     — Read, Notify
```

#### 3. Enable Notifications

1. Find characteristic **0xFF03** (Status)
2. Tap the **⬇ download arrow** (or triple-down-arrow icon) to enable notifications
3. You should see "Notifications enabled"

This is how the glasses will tell you about OTA progress.

#### 4. Enter OTA Mode

1. Find characteristic **0xFF01** (Control)
2. Tap the **⬆ upload arrow** (write icon)
3. Select type: **BYTE ARRAY**
4. Enter: **FD**
5. Tap **SEND**

Watch the notification on 0xFF03. You should receive: `01` (OTA_STATUS_READY)

The lenses will go clear (PWM stops during OTA).

#### 5. Send the Firmware File

1. Find characteristic **0xFF02** (OTA Data)
2. Tap the **⬆ upload arrow**
3. In nRF Connect, look for the option to **send a file** or **stream data**
   - On Android: tap the write icon → select "Write (file)" or use the "Write" option then select your .bin file
   - The app may show a file picker — select your **ESP32_Ble.bin**
4. Set write type to **Write Without Response** (faster)
5. Set packet size / MTU to the maximum available (typically 244 or 509 bytes)
6. Send the file

**Alternative method if nRF Connect doesn't have file streaming:**

Use the **nRF Connect → Macros** feature or a script. The simplest
approach on Android:

1. Tap the write button on 0xFF02
2. Select "Send file" (available in newer nRF Connect versions)
3. Choose your .bin file
4. It will automatically chunk and send the file

During transfer, you'll see progress notifications on 0xFF03:
- `02 XX XX XX` = Progress (XX XX XX = bytes received, 3 bytes big-endian)

A ~200KB firmware takes about 30-90 seconds over BLE.

#### 6. Finish the Update

After the entire file is sent:

1. Go back to characteristic **0xFF01** (Control)
2. Write: **FE** (BYTE ARRAY)
3. Tap **SEND**

You should see notification `03` (OTA_STATUS_SUCCESS) on 0xFF03.

The glasses will reboot in ~2 seconds with the new firmware.

#### 7. Reconnect and Verify

1. The glasses will disconnect during reboot
2. Wait 3-5 seconds
3. Scan and reconnect in nRF Connect
4. The glasses should be running the new firmware

### Cancel an OTA in Progress

If something goes wrong during transfer:
1. Write **FF** to characteristic 0xFF01
2. The OTA will be safely cancelled
3. The glasses continue running the current firmware (no damage)

---

## Quick Reference Card

| Action | Characteristic | Value | Response (0xFF03) |
|--------|---------------|-------|-------------------|
| Enter OTA mode | 0xFF01 | FD | 01 (ready) |
| Send firmware data | 0xFF02 | [binary chunks] | 02 XX XX XX (progress) |
| Finish OTA | 0xFF01 | FE | 03 (success, rebooting) |
| Cancel OTA | 0xFF01 | FF | 05 (cancelled) |
| Normal lens command | 0xFF01 | [duty] [time] | (none) |

| Status Notification | Meaning |
|--------------------|---------|
| 01 | OTA ready for data |
| 02 XX XX XX | Progress (bytes received) |
| 03 | Success, rebooting |
| 04 XX | Error (XX = error code) |
| 05 | Cancelled |

| Error Code | Meaning |
|------------|---------|
| 01 | OTA begin failed |
| 02 | Flash write failed |
| 03 | OTA finalize failed |
| 04 | Not in OTA mode |
| 05 | No update partition found |

---

## Troubleshooting

**"No update partition found" error:**
The partition table wasn't updated. Re-flash with ERASE, then flash
all 3 binaries (bootloader + partition table + app).

**Transfer seems stuck:**
BLE can be slow. A 200KB file at ~10KB/s takes ~20 seconds.
Check that Write Without Response is selected (not Write With Response).

**Glasses don't reboot after finish:**
The binary might be corrupt. Cancel (write FF) and try again.
Make sure you're sending the correct .bin file (ESP32_Ble.bin, not
bootloader.bin or the ELF file).

**Glasses are bricked after OTA:**
Not possible! If the new firmware is bad, the bootloader will keep
trying to boot from the last working partition. Worst case, use the
wired programmer to flash again.

**Can't connect after OTA:**
Power cycle the glasses (close and open the arms to trigger
sleep/wake cycle, or disconnect and reconnect battery).

**nRF Connect doesn't have "send file" option:**
Use **nRF Toolbox** app instead (also free from Nordic). It has a
dedicated UART/DFU tool that can stream files. Or on a computer,
use the nRF Connect Desktop app which supports file streaming.

---

## Partition Layout (4MB Flash)

| Name | Offset | Size | Purpose |
|------|--------|------|---------|
| nvs | 0x009000 | 16 KB | Non-volatile storage |
| otadata | 0x00D000 | 8 KB | OTA boot selection data |
| phy_init | 0x00F000 | 4 KB | PHY calibration data |
| ota_0 | 0x010000 | 1536 KB | App slot 0 (initial flash goes here) |
| ota_1 | 0x190000 | 1536 KB | App slot 1 (first OTA writes here) |

The bootloader alternates between ota_0 and ota_1. Each slot is 1.5MB,
plenty of room for the firmware (~200-300KB typically).
