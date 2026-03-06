# Smart Glasses v4.2 - BLE OTA Update Guide

## Quick Summary

**OTA Commands (write to 0xFF01):**
| Bytes | Action |
|-------|--------|
| `A8 00` | Start OTA mode |
| `A9 00` | Finish OTA (validate + reboot) |
| `AA 00` | Cancel OTA |

**Firmware data:** Write chunks to 0xFF02

**Progress:** Subscribe to notifications on 0xFF03

---

## First-Time Setup (Wired Flash Required)

The **first** OTA-enabled firmware must be flashed via USB programmer:

1. Copy `main_v4.2.c` to your project's `main/main.c`
2. Ensure `partitions_ota.csv` is in your project root
3. Ensure `sdkconfig` has:
   - `CONFIG_ESPTOOLPY_FLASHSIZE_4MB=y`
   - `CONFIG_PARTITION_TABLE_CUSTOM=y`
   - `CONFIG_PARTITION_TABLE_CUSTOM_FILENAME="partitions_ota.csv"`

4. Build:
   ```
   idf.py fullclean
   idf.py build
   ```

5. Flash with **ERASE** (this installs the new partition table):
   | File | Address |
   |------|---------|
   | bootloader.bin | 0x1000 |
   | partition-table.bin | 0x8000 |
   | ESP32_Ble.bin | 0x10000 |

After this, all future updates are wireless.

---

## OTA Update via nRF Connect

### Prerequisites
- nRF Connect app (iOS/Android)
- New firmware `.bin` file on your phone

### Step-by-Step

#### 1. Connect
- Open nRF Connect → SCAN
- Find "Smart_Glasses" → CONNECT

#### 2. Enable Notifications
- Find characteristic **0xFF03** (Status)
- Tap the **⬇** icon to enable notifications

#### 3. Start OTA Mode
- Find characteristic **0xFF01** (Control)
- Tap **⬆** (write)
- Type: `A8` or `A800` (byte array)
- Send

You should see notification `01` (ready).  
Lenses go clear, breathing/strobe pauses.

#### 4. Send Firmware
- Find characteristic **0xFF02** (OTA Data)
- Write your `.bin` file in chunks
- Use "Write Without Response" for speed
- Set max packet size (244 or 509 bytes)

Progress notifications: `02 XX XX XX` (bytes received)

#### 5. Finish OTA
- Go back to **0xFF01**
- Write: `A9` or `A900`
- Send

Success notification: `03`  
Glasses reboot in ~2 seconds.

#### 6. Reconnect
- Wait 3-5 seconds
- Scan and reconnect
- Verify new firmware is running

### Cancel OTA
If something goes wrong:
- Write `AA` or `AA00` to 0xFF01
- Notification: `05` (cancelled)
- Glasses resume normal operation

---

## BLE Characteristics

| UUID | Name | Properties | Purpose |
|------|------|------------|---------|
| 0xFF01 | Control | Read, Write | Commands (lens + OTA) |
| 0xFF02 | OTA Data | Write, WriteNR | Firmware binary chunks |
| 0xFF03 | Status | Read, Notify | OTA progress feedback |

---

## All Commands (0xFF01)

### Legacy (single byte)
| Byte | Effect |
|------|--------|
| 0x00 | Clear (0% duty) |
| 0x80 | 50% duty |
| 0xFF | Full dark (100% duty) |

### Extended (multi-byte)
| Command | Bytes | Effect |
|---------|-------|--------|
| Strobe | `A1 [start_hz] [end_hz]` | Enable strobe mode |
| Brightness | `A2 [0-100]` | Set max brightness |
| Breathing | `A3 [inh] [hold_in] [exh] [hold_out]` | Breathing (no strobe) |
| Session | `A4 [minutes]` | Set duration (1-60) |
| Override | `A5 [duty]` | Static duty, stop animation |
| Resume | `A6 00` | Restart session |
| Sleep | `A7 00` | Enter deep sleep |
| OTA Start | `A8 00` | Enter OTA mode |
| OTA Finish | `A9 00` | Validate + reboot |
| OTA Cancel | `AA 00` | Abort OTA |

---

## Status Notifications (0xFF03)

| Value | Meaning |
|-------|---------|
| `01` | OTA ready for data |
| `02 XX XX XX` | Progress (3-byte count) |
| `03` | Success, rebooting |
| `04 XX` | Error (XX = code) |
| `05` | Cancelled |

### Error Codes
| Code | Meaning |
|------|---------|
| 01 | OTA begin failed |
| 02 | Flash write failed |
| 03 | Validation failed |
| 04 | Not in OTA mode |
| 05 | No update partition |

---

## Troubleshooting

**"No update partition" error:**  
Partition table not updated. Re-flash with ERASE via wired programmer.

**Transfer stuck:**  
~200KB firmware takes 30-90 seconds. Use "Write Without Response".

**Glasses don't reboot:**  
Binary may be corrupt. Cancel (`AA`) and retry with correct file.

**Can't connect after OTA:**  
Power cycle (close/open arms) or wait 10 seconds.

**Lost OTA capability:**  
You flashed firmware without OTA code. Use wired programmer to restore.

---

## Partition Layout (4MB)

| Name | Offset | Size | Purpose |
|------|--------|------|---------|
| nvs | 0x9000 | 16KB | Settings storage |
| otadata | 0xD000 | 8KB | Boot partition selector |
| phy_init | 0xF000 | 4KB | RF calibration |
| ota_0 | 0x10000 | 1.5MB | App slot 0 |
| ota_1 | 0x190000 | 1.5MB | App slot 1 |

Bootloader alternates between ota_0 and ota_1 on each OTA update.
