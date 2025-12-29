# Image Files Needed for AW/BW Support

## Directory
Place all images in: `sidecar/www/dvi-card/dvi-lv/`

## Required Images

### Base Diagrams
- [x] `aw.gif` - Rename existing `dvi.gif` to this
- [ ] `bw.gif` - New diagram for Brine-to-Water/Geothermal pumps

### Common Overlays (used by both AW and BW)
- [x] `CV_on.gif` - Central heating pump (no changes)
- [x] `CVflow_on.gif` - Central heating flow (no changes)

### AW-Specific Overlays
- [x] `COMP_on.gif` - Compressor for Air-to-Water
- [x] `HP_on.gif` - Heat pump / evaporator loop for Air-to-Water

### BW-Specific Overlays  
- [ ] `COMP_bw_on.gif` - Compressor for Brine-to-Water (new file needed)
- [ ] `brine_on.gif` - Brine/geothermal loop (replaces HP_on.gif for BW)

## Implementation Status

✅ **Code Changes Complete:**
- bridge.py: Exports pump_type in state payload
- webbridge.py: Exposes pump_type via `/api/pump_type` endpoint
- diagram.js: Dynamically selects images based on pump type
- dvi-lv-heatpump-card.js: Fetches pump type on initialization

## TODO - Image Files
1. Rename `dvi.gif` → `aw.gif`
2. Create `bw.gif` (base diagram for geothermal pumps)
3. Create `COMP_bw_on.gif` (compressor overlay for BW)
4. Create `brine_on.gif` (geothermal loop overlay)

## Testing Checklist
- [x] AW pump: Displays aw.gif with COMP_on.gif and HP_on.gif
- [ ] BW pump: Should display bw.gif with COMP_bw_on.gif and brine_on.gif
- [ ] API endpoint /api/pump_type returns correct type
- [ ] Dynamic image loading works correctly
