# AW/BW Pump Type Detection Implementation

## Summary
Successfully implemented automatic pump type detection and element filtering for DVI heat pumps based on FC06 register 0x15.

## Pump Types
- **AW (Air-to-Water)**: Register 0x15 = 0
- **BW (Brine-to-Water/Geothermal)**: Register 0x15 = 1

## Changes Made

### 1. Pump Type Detection Function
Added `detect_pump_type()` function that:
- Reads FC06 register 0x15 at startup
- Returns "AW" for value 0, "BW" for value 1
- Falls back to AW configuration if detection fails

### 2. Coil Definitions (FC01)
**AW Type** (`coil_names_aw`): All coils including:
- Coil 1: "3-Way shunt VV open/close"
- Coil 5: "El-tracing CV/drain" 
- Coil 8: "4-way valve defrost"
- Coil 9: "Liquid injection solenoid valve"
- Coil 14: "Sum alarm failure"

**BW Type** (`coil_names_bw`): Excludes AW-only coils, renames coil 8:
- Coil 1: EXCLUDED (AW-only)
- Coil 5: NOT USED
- Coil 8: "Circ. pump geothermal" (renamed)
- Coil 9: NOT USED
- Coil 14: NOT USED

### 3. Temperature Sensor Definitions (FC04)
**AW Type** (`fc04_labels_aw`): All sensors including:
- Sensor 0x06: "Evaporator"

**BW Type** (`fc04_labels_bw`): Excludes evaporator, adds geothermal sensors:
- Sensor 0x06: EXCLUDED (AW-only)
- Sensor 0x0D (0x13): "Cold side warm" (BW-specific)
- Sensor 0x0E (0x14): "Cold side cold" (BW-specific)

**Note**: "Storage tank VV" (sensor 0x03) is valid for BOTH types - VV refers to domestic hot water, not pump type.

### 4. FC06 Registers
- ALL FC06 registers remain valid for both pump types
- No filtering applied to FC06 registers

### 5. Filter Functions
Added three filter functions:
- `filter_coils_by_type(pump_type)`: Returns applicable coils
- `filter_fc04_by_type(pump_type)`: Returns applicable sensors
- Both return AW configuration if pump type is unknown

### 6. Updated Functions
Modified the following functions to use filtered elements:
- `read_coils()`: Filters coils based on detected type
- `publish_all_discovery()`: Only publishes applicable entities to Home Assistant
- Main loop FC04 reading: Only reads applicable sensors
- History sampling: Only samples applicable sensors

### 7. Startup Sequence
1. Detect pump type from register 0x15
2. Print detected type with description
3. Connect to MQTT
4. Publish filtered discovery configs to Home Assistant
5. Start main monitoring loop with filtered elements

## Benefits
1. **Automatic Detection**: No manual configuration needed
2. **Clean UI**: Home Assistant only shows relevant entities
3. **Correct Naming**: BW pumps see "Circ. pump geothermal" instead of "4-way valve defrost"
4. **Geothermal Support**: BW pumps can monitor geothermal loop temperatures
5. **Backward Compatible**: Falls back to AW configuration if detection fails

## Testing
- Syntax check: ✅ PASSED
- No lint errors: ✅ CONFIRMED
- File structure: ✅ VALID

## Files Modified
- `bridge.py`: Main implementation
- `apply_aw_bw_changes.py`: Patcher script (can be deleted after use)
- `AW_BW_changes.txt`: Change documentation

## Next Steps
1. Test on actual AW heatpump hardware
2. Test on actual BW heatpump hardware  
3. Verify correct sensor readings for geothermal loops
4. Verify correct coil status for geothermal circulation pumps
