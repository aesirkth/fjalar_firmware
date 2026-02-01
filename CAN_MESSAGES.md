# CAN Bus Communication Protocol

This document describes the CAN messages used for external communication with the Fjalar flight controller. This guide is intended for teams connecting their electronics to the flight controller via CAN bus.

## Critical Messages for Fjalar Operation

**For Fjalar's core functionality, only ONE CAN message is critical:**

### ✅ **CRITICAL: Fjalar to Loki (TX) - CAN ID 0x67F/0x57F**

This message is **essential** for Fjalar's operation because:
- It transmits the calculated airbrake angle from Fjalar's PID control loop to Loki
- Without this message, airbrake control cannot function
- The airbrake angle is computed at 100 Hz in the control thread and must be sent to Loki for actuation
- This is the primary control output for altitude control during the coast phase

**All other CAN messages are NOT critical for Fjalar's functioning:**
- **Loki to Fjalar (RX, 0x6FF)**: Only used for telemetry/logging, not used in any control loops
- **All GCS messages**: Telemetry only, or currently handled via LoRa communication

---

## Implementation Requirements Analysis

### ✅ **Already Implemented - No Action Needed**

#### 1. Fjalar to Loki (TX, 0x67F/0x57F) - **IMPLEMENTED** ✅
- **Status**: Fully functional
- **Why it exists**: Critical for airbrake control
- **Action**: None - already working

#### 2. Loki to Fjalar (RX, 0x6FF) - **IMPLEMENTED** ✅
- **Status**: Fully functional
- **Why it exists**: Telemetry/monitoring of Loki status
- **Action**: None - already working (optional but implemented)

---

### ⚠️ **Should Be Implemented - Recommended**

#### 3. GCS to Fjalar: Ready/Arm (RX, 0x700) - **NOT IMPLEMENTED** ❌
- **Priority**: **MEDIUM** - Recommended for redundancy
- **Why implement**:
  - Currently handled via LoRa (`LORA_READY_INITIATE_FJALAR`)
  - CAN provides faster, more reliable communication for critical commands
  - Redundancy: If LoRa fails, CAN can still arm the system
  - Lower latency: CAN is faster than LoRa for time-critical operations
- **Use case**: Ground operations need reliable arming mechanism
- **Implementation effort**: Low (simple boolean flag)

#### 4. GCS to Fjalar: Launch (RX, 0x701) - **NOT IMPLEMENTED** ❌
- **Priority**: **MEDIUM** - Recommended for redundancy
- **Why implement**:
  - Currently handled via LoRa (`LORA_READY_LAUNCH_FJALAR`)
  - Critical command that benefits from redundant communication paths
  - CAN provides deterministic timing vs. LoRa's variable latency
  - Safety: Multiple communication paths for launch command
- **Use case**: Launch operations require reliable command delivery
- **Implementation effort**: Low (simple boolean flag)

---

### 📊 **Nice to Have - Optional Telemetry**

#### 5. Fjalar to GCS: Flight Status (TX, 0x720) - **NOT IMPLEMENTED** ❌
- **Priority**: **LOW** - Optional
- **Why implement**:
  - Provides consolidated status message for ground monitoring
  - Includes flight state, Loki status, deployment status, GNSS fix
  - Useful for ground operations dashboard
- **Why NOT critical**:
  - Similar data already available via LoRa (flight state, GNSS)
  - Not used in any control loops
  - Telemetry only - doesn't affect flight functionality
- **Implementation effort**: Medium (multiple data fields)

#### 6. Fjalar to GCS: Other Telemetry Messages (TX, 0x721-0x731) - **NOT IMPLEMENTED** ❌
- **Priority**: **LOW** - Optional
- **Why implement**:
  - Comprehensive telemetry for ground monitoring
  - Useful for post-flight analysis and real-time monitoring
  - Helps with debugging and system health monitoring
- **Why NOT critical**:
  - All telemetry data - doesn't affect flight control
  - Much of this data may already be logged to flash
  - Ground operations can function without real-time telemetry
  - LoRa already provides basic telemetry (GPS, flight state)
- **Implementation effort**: High (many messages, various data types)

**Specific telemetry messages:**
- **0x721** (Fafnir Status): Only needed if Fafnir system is used
- **0x722** (Thrust): Only needed if loadcell data is critical for ground ops
- **0x723** (Airbrake Status): Redundant with data in 0x67F message
- **0x724** (Pyro Status): Useful for safety monitoring
- **0x725-0x72A** (Acceleration/Velocity): Already in state estimate, logged to flash
- **0x72B-0x72C** (Attitude): Already in state estimate, logged to flash
- **0x72D-0x72E** (Position): Already sent via LoRa
- **0x72F-0x730** (Sigurd Temperatures): Only needed if Sigurd system is used
- **0x731** (Battery Voltages): Useful for system health monitoring

---

### ❌ **Does NOT Need to Be Implemented - Why Not**

#### USB CAN-over-USB Protocol - **NOT IMPLEMENTED** ❌
- **Priority**: **LOW** - Not recommended
- **Why NOT implement**:
  - Current USB implementation uses nanopb/protobuf protocol (different format)
  - USB is already functional for data logging and commands
  - CAN-over-USB would require significant refactoring
  - No clear benefit over existing USB protocol
  - Ground station can use existing USB protocol or LoRa
- **Alternative**: Use existing USB CDC ACM with nanopb protocol

---

## Implementation Priority Summary

| Message | Priority | Reason | Effort |
|---------|----------|--------|--------|
| **0x67F/0x57F** (Fjalar→Loki TX) | ✅ DONE | Critical for airbrake control | - |
| **0x6FF** (Loki→Fjalar RX) | ✅ DONE | Telemetry (optional) | - |
| **0x700** (GCS→Fjalar: Ready/Arm) | ⚠️ MEDIUM | Redundancy for critical command | Low |
| **0x701** (GCS→Fjalar: Launch) | ⚠️ MEDIUM | Redundancy for critical command | Low |
| **0x720** (Fjalar→GCS: Status) | 📊 LOW | Telemetry only | Medium |
| **0x721-0x731** (Fjalar→GCS: Telemetry) | 📊 LOW | Telemetry only | High |
| **USB CAN Protocol** | ❌ SKIP | Existing USB protocol works | High |

---

## Recommended Implementation Plan

### Phase 1: Critical Redundancy (Recommended)
1. Implement **0x700** (Ready/Arm) - Provides CAN backup for LoRa command
2. Implement **0x701** (Launch) - Provides CAN backup for LoRa command

**Benefits**: 
- Redundant communication paths for safety-critical commands
- Lower latency than LoRa
- Simple implementation (boolean flags)

### Phase 2: Essential Telemetry (Optional)
3. Implement **0x720** (Flight Status) - Consolidated status for ground ops

**Benefits**:
- Single message with key status information
- Useful for ground operations dashboard

### Phase 3: Comprehensive Telemetry (Optional)
4. Implement remaining telemetry messages (0x721-0x731) based on operational needs

**Considerations**:
- Only implement messages for systems that are actually used (Fafnir, Sigurd, etc.)
- Evaluate if data is already available via other channels (LoRa, flash logs)
- Balance implementation effort vs. operational benefit

---

## CAN Bus Configuration

- **Bitrate**: 500 kbps (nominal and data)
- **Frame Format**: Standard 11-bit CAN IDs
- **Transmission Rate**: 100 Hz (10 ms period)

---

## Messages Transmitted by Flight Controller (TX)

### Fjalar to Loki

**CAN ID**: 
- `0x67F` for Fjalar 1
- `0x57F` for Fjalar 2

**DLC**: 4 bytes

**Transmission Rate**: 100 Hz (every 10 ms)

**Data Format**:

| Byte | Description | Format | Range/Values |
|------|-------------|--------|--------------|
| 0 | High nibble: Event marker<br>Low nibble: Flight state | High nibble: `0xA` (if event_above_acs_threshold) or `0x5` (otherwise)<br>Low nibble: Flight state enum (0-8) | High: `0x5` or `0xA`<br>Low: 0-8 |
| 1 | Event marker | `0xAA` (if event_above_acs_threshold AND STATE_COAST)<br>`0x55` (otherwise) | `0x55` or `0xAA` |
| 2-3 | Airbrake angle | 16-bit unsigned integer, big-endian<br>Angle × 100 | 0-36000 (0.0° - 360.0°) |

**Flight State Enum Values**:
- `0` = STATE_IDLE
- `1` = STATE_AWAITING_INIT
- `2` = STATE_INITIATED
- `3` = STATE_AWAITING_LAUNCH
- `4` = STATE_BOOST
- `5` = STATE_COAST
- `6` = STATE_DROGUE_DESCENT
- `7` = STATE_MAIN_DESCENT
- `8` = STATE_LANDED

**Example Decoding**:
```c
// Byte 0
uint8_t event_marker = (data[0] >> 4) & 0x0F;  // High nibble
uint8_t flight_state = data[0] & 0x0F;          // Low nibble

// Byte 1
uint8_t event_flag = data[1];  // 0xAA or 0x55

// Bytes 2-3: Airbrake angle
uint16_t raw_angle = (data[2] << 8) | data[3];
float airbrake_angle = raw_angle / 100.0f;  // Convert to degrees
```

---

## Messages Received by Flight Controller (RX)

### Loki to Fjalar

**CAN ID**: `0x6FF`

**DLC**: 4 bytes

**Data Format**:

| Byte | Description | Format | Range/Values |
|------|-------------|--------|--------------|
| 0 | Low nibble: State<br>High nibble: Sub-state | Low nibble: State value<br>High nibble: Sub-state value | 0-15 each |
| 1-2 | Angle | 16-bit unsigned integer, big-endian<br>Angle × 100 | 0-65535 (0.0° - 655.35°) |
| 3 | Battery voltage | 8-bit unsigned integer<br>Voltage × 10 | 0-255 (0.0V - 25.5V) |

**Example Encoding**:
```c
// Byte 0
uint8_t state = loki_state & 0x0F;           // Low nibble
uint8_t substate = loki_substate & 0x0F;     // High nibble
data[0] = (substate << 4) | state;

// Bytes 1-2: Angle
uint16_t raw_angle = (uint16_t)roundf(loki_angle * 100.0f);
data[1] = (raw_angle >> 8) & 0xFF;  // MSB
data[2] = raw_angle & 0xFF;         // LSB

// Byte 3: Battery voltage
data[3] = (uint8_t)roundf(loki_battery_voltage * 10.0f);
```

**Example Decoding** (as implemented in flight controller):
```c
// Byte 0
uint8_t loki_state = data[0] & 0x0F;
uint8_t loki_sub_state = (data[0] >> 4) & 0x0F;

// Bytes 1-2: Angle
uint16_t raw_angle = (data[1] << 8) | data[2];
float loki_angle = raw_angle / 100.0f;

// Byte 3: Battery voltage
float loki_battery_voltage = data[3] / 10.0f;
```

---

## Ground Control Station (GCS) Messages

The following messages are specified for communication with the Ground Control Station. **All GCS messages are currently NOT IMPLEMENTED** in the flight controller firmware. These messages are documented here for reference and future implementation.

### GCS to Fjalar (RX - Commands)

These messages are received by the flight controller from the Ground Control Station.

#### Ready/Arm Fjalar

**CAN ID**: `0x700`  
**Status**: ❌ **NOT IMPLEMENTED**

| Bit | Description | Type | Values |
|-----|-------------|------|--------|
| 0 | Ready/arm Fjalar | bool | `0` = not ready, `1` = ready/armed |

**Note**: Currently, the ready/arm functionality is implemented via LoRa communication (`LORA_READY_INITIATE_FJALAR`), not CAN.

#### Launch Command

**CAN ID**: `0x701`  
**Status**: ❌ **NOT IMPLEMENTED**

| Bit | Description | Type | Values |
|-----|-------------|------|--------|
| 0 | Launch command | bool | `0` = no launch, `1` = launch |

**Note**: Currently, the launch command is implemented via LoRa communication (`LORA_READY_LAUNCH_FJALAR`), not CAN.

### Fjalar to GCS (TX - Telemetry)

These messages are transmitted by the flight controller to the Ground Control Station.

#### Flight Status

**CAN ID**: `0x720`  
**Status**: ❌ **NOT IMPLEMENTED**

| Byte | Description | Type | Range/Values |
|------|-------------|------|--------------|
| 0 | Flight state | uint8 | 0-8 (see Flight State Enum) |
| 1 | Loki state | uint8 | 0-15 |
| 2 | Loki substate | uint8 | 0-15 |
| 3 | Drogue deployed | bool | `0` = not deployed, `1` = deployed |
| 4 | Main deployed/line cut | bool | `0` = not deployed, `1` = deployed |
| 5 | GNSS fix | bool | `0` = no fix, `1` = fix acquired |

#### Fafnir Status

**CAN ID**: `0x721`  
**Status**: ❌ **NOT IMPLEMENTED**

| Byte | Description | Type | Format |
|------|-------------|------|--------|
| 0-3 | Fafnir main valve percentage | float32 | IEEE 754 single precision |
| 4 | Fafnir motor solenoid 1 | bool | `0` = closed, `1` = open |
| 5 | Fafnir motor solenoid 2 | bool | `0` = closed, `1` = open |
| 6 | Fafnir motor solenoid 3 | bool | `0` = closed, `1` = open |
| 7 | Fafnir motor solenoid 4 | bool | `0` = closed, `1` = open |

#### Thrust Data

**CAN ID**: `0x722`  
**Status**: ❌ **NOT IMPLEMENTED**

| Byte | Description | Type | Format |
|------|-------------|------|--------|
| 0-3 | Thrust (from loadcell) | float32 | IEEE 754 single precision |

#### Airbrake Status

**CAN ID**: `0x723`  
**Status**: ❌ **NOT IMPLEMENTED**

| Byte | Description | Type | Format |
|------|-------------|------|--------|
| 0 | Freyr airbrake safety solenoid | bool | `0` = closed, `1` = open |
| 1-4 | Airbrake percentage | float32 | IEEE 754 single precision |

#### Pyro Status

**CAN ID**: `0x724`  
**Status**: ❌ **NOT IMPLEMENTED**

| Byte | Bit | Description | Type | Values |
|------|-----|-------------|------|--------|
| 0 | 0 | Pyro1 fired/connected | bool | `0` = not fired/not connected, `1` = fired/connected |
| 0 | 1 | Pyro2 fired/connected | bool | `0` = not fired/not connected, `1` = fired/connected |
| 0 | 2 | Pyro3 fired/connected | bool | `0` = not fired/not connected, `1` = fired/connected |

#### Acceleration Data

**CAN ID**: `0x725` - X and Y acceleration  
**Status**: ❌ **NOT IMPLEMENTED**

| Byte | Description | Type | Format |
|------|-------------|------|--------|
| 0-3 | ax (X acceleration) | float32 | IEEE 754 single precision |
| 4-7 | ay (Y acceleration) | float32 | IEEE 754 single precision |

**CAN ID**: `0x726` - Z acceleration  
**Status**: ❌ **NOT IMPLEMENTED**

| Byte | Description | Type | Format |
|------|-------------|------|--------|
| 0-3 | az (Z acceleration) | float32 | IEEE 754 single precision |

#### Velocity Data

**CAN ID**: `0x727` - X and Y velocity  
**Status**: ❌ **NOT IMPLEMENTED**

| Byte | Description | Type | Format |
|------|-------------|------|--------|
| 0-3 | vx (X velocity) | float32 | IEEE 754 single precision |
| 4-7 | vy (Y velocity) | float32 | IEEE 754 single precision |

**CAN ID**: `0x72A` - Z velocity  
**Status**: ❌ **NOT IMPLEMENTED**

| Byte | Description | Type | Format |
|------|-------------|------|--------|
| 0-3 | vz (Z velocity) | float32 | IEEE 754 single precision |

#### Attitude Data

**CAN ID**: `0x72B` - Roll and Pitch  
**Status**: ❌ **NOT IMPLEMENTED**

| Byte | Description | Type | Format |
|------|-------------|------|--------|
| 0-3 | Roll | float32 | IEEE 754 single precision |
| 4-7 | Pitch | float32 | IEEE 754 single precision |

**CAN ID**: `0x72C` - Yaw  
**Status**: ❌ **NOT IMPLEMENTED**

| Byte | Description | Type | Format |
|------|-------------|------|--------|
| 0-3 | Yaw | float32 | IEEE 754 single precision |

#### Position Data

**CAN ID**: `0x72D` - Longitude and Latitude  
**Status**: ❌ **NOT IMPLEMENTED**

| Byte | Description | Type | Format |
|------|-------------|------|--------|
| 0-3 | Longitude | float32 | IEEE 754 single precision |
| 4-7 | Latitude | float32 | IEEE 754 single precision |

**CAN ID**: `0x72E` - Altitude  
**Status**: ❌ **NOT IMPLEMENTED**

| Byte | Description | Type | Format |
|------|-------------|------|--------|
| 0-3 | Altitude | float32 | IEEE 754 single precision |

#### Temperature Data

**CAN ID**: `0x72F` - Sigurd Temperatures 1 and 2  
**Status**: ❌ **NOT IMPLEMENTED**

| Byte | Description | Type | Format |
|------|-------------|------|--------|
| 0-3 | Sigurd temperature 1 | float32 | IEEE 754 single precision |
| 4-7 | Sigurd temperature 2 | float32 | IEEE 754 single precision |

**CAN ID**: `0x730` - Sigurd Temperatures 3 and 4  
**Status**: ❌ **NOT IMPLEMENTED**

| Byte | Description | Type | Format |
|------|-------------|------|--------|
| 0-3 | Sigurd temperature 3 | float32 | IEEE 754 single precision |
| 4-7 | Sigurd temperature 4 | float32 | IEEE 754 single precision |

#### Battery Voltage

**CAN ID**: `0x731` - Battery Voltages  
**Status**: ❌ **NOT IMPLEMENTED**

| Byte | Description | Type | Format |
|------|-------------|------|--------|
| 0-3 | Fjalar battery voltage | float32 | IEEE 754 single precision |
| 4-7 | Loki battery voltage | float32 | IEEE 754 single precision |

---

## USB Message Specification

The flight controller can communicate CAN messages over USB CDC ACM. CAN packets are packaged with a header and sent over USB.

**Note**: The current USB implementation uses a different protocol (nanopb/protobuf). The CAN-over-USB format described below is **NOT IMPLEMENTED** in the current firmware.

### USB Packet Format

| Byte | Data | Description |
|------|------|-------------|
| 0 | `0xAA` | Header byte 1 |
| 1 | `0xAA` | Header byte 2 |
| 2-9 | x | Milliseconds since 1970-01-01 UTC (64-bit timestamp) |
| 10 | x | Packet type (CAN ID - 0x700) |
| 11 | x | Length of CAN packet (1-8 bytes) |
| 12-19 | x | CAN packet data (padded if necessary) |

**Notes**:
- The CAN packet size is between 1 and 8 bytes
- The timestamp is a 64-bit value representing milliseconds since Unix epoch (1970-01-01 00:00:00 UTC)
- The packet type is calculated as `CAN_ID - 0x700`
- For example, CAN ID `0x720` would have packet type `0x20` (0x720 - 0x700)

---

## Message Filtering

The flight controller uses CAN filters to receive messages:

- **Loki filter**: ID `0x6FF` with mask `0x7FF` (matches all 11 bits)
- **GCS filters**: Not yet implemented (will need filters for IDs `0x700` and `0x701`)

---

## Notes

- All multi-byte values use **big-endian** (network byte order), except float32 values which use IEEE 754 format
- The flight controller transmits at 100 Hz (every 10 ms) for Loki messages
- Ensure your CAN transceiver is configured for 500 kbps
- Standard 11-bit CAN IDs are used (not extended 29-bit)
- The flight controller validates DLC (Data Length Code) and will log errors if incorrect
- Currently, ready/arm and launch commands are handled via LoRa communication, not CAN

---

## Future Messages (Not Yet Implemented)

The following messages are defined in the protocol schema but not yet implemented in the flight controller:

- **Sigurd to Fjalar**: Sensor data (ID and format TBD)
- **Fjalar to Sigurd**: Control commands (ID and format TBD)
- **Fafnir to Fjalar**: Status/telemetry (ID and format TBD)
- **Fjalar to Fafnir**: Solenoid control commands (ID and format TBD)

These will be documented once implemented.
