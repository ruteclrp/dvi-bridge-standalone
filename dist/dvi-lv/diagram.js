const CV_NIGHT_ICONS = {
	Timer: "mdi:clock-outline",
	"Constant day": "mdi:weather-sunny",
	"Constant night": "mdi:weather-night",
};

const VV_SCHEDULE_ICONS = {
	Timer: "mdi:clock-outline",
	"Constant on": "mdi:toggle-switch",
	"Constant off": "mdi:toggle-switch-off-outline",
};

const CHIP_TITLES = {
	info: "Information",
	cv: "Centralvarme",
	vv: "Varmtvandstemperatur",
	aux: "El-patron / AUX",
};

const opacityFor = (state) => (state === "on" ? 1 : 0.25);

export function buildDiagramView({ hass, config, imageBase, pumpType }) {
	// Determine pump type (default to AW if not provided)
	const isPumpTypeBW = pumpType === "BW";
	
	// Select correct images based on pump type
	const baseDiagram = isPumpTypeBW ? "bw.gif" : "aw.gif";
	const compressorImage = isPumpTypeBW ? "comp_bw_on.gif" : "COMP_on.gif";
	const evapLoopImage = isPumpTypeBW ? "brine_on.gif" : "HP_on.gif";
	
	const getState = (entityId) =>
		entityId && hass.states[entityId] ? hass.states[entityId].state : null;

	const getUnit = (entityId) =>
		entityId && hass.states[entityId]
			? hass.states[entityId].attributes.unit_of_measurement || ""
			: "";

	// Helper function to find entity by name (searches both 'name' and 'friendly_name' attributes)
	const findEntityByName = (searchName) => {
		if (!searchName) return null;
		for (const entityId in hass.states) {
			const entity = hass.states[entityId];
			// Check if searchName matches or is contained in 'name' or 'friendly_name' attributes
			// This handles both sidecar (exact match) and HA (with device prefix like "DVI LV12 Circ. pump CV")
			const name = entity.attributes?.name;
			const friendlyName = entity.attributes?.friendly_name;
			if (name === searchName || friendlyName === searchName ||
			    (friendlyName && friendlyName.includes(searchName))) {
				return entityId;
			}
		}
		return null;
	};

	const getStateByName = (searchName) => {
		const entityId = findEntityByName(searchName);
		return entityId ? getState(entityId) : null;
	};

	const showUnit = config.show_temp_unit ?? false;

	const stateEntityMap = {
		outdoor: config.outdoor_temp,
		curve: config.curve_temp,
		tankCv: config.storage_tank_cv,
		tankVv: config.storage_tank_vv,
		evap: config.evaporator_temp,
		coldSideWarm: config.cold_side_warm_temp,
		coldSideCold: config.cold_side_cold_temp,
		hp: config.hp_temp,
		lp: config.lp_temp,
		cvForward: config.cv_forward_temp,
		cvReturn: config.cv_return_temp,
	};

	const unitForKey = (key) => {
		if (!showUnit) return "";
		const eid = stateEntityMap[key];
		if (!eid) return "";
		const unit = getUnit(eid);
		return unit ? ` ${unit}` : "";
	};

	const valueWithUnit = (key, value) =>
		value === null ? "" : `${value}<span class="diagram-unit">${unitForKey(key)}</span>`;

	const normalizeState = (state) => String(state).toLowerCase();
	const isModeActive = (state, schedule = null) => {
		const normalized = normalizeState(state);
		if (normalized === "off" || normalized === "0") return false;
		if (normalized === "on" || normalized === "1") return true;
		if (normalized === "automatic") return true;
		if (normalized === "backup operation") return true;
		if (normalized === "timer" || normalized === "2") {
			if (schedule) {
				const schedNorm = normalizeState(schedule);
				return schedNorm !== "constant off" && schedNorm !== "0";
			}
			return true;
		}
		return false;
	};
	const chipStateClass = (active) => (active ? "mode-chip--active" : "mode-chip--inactive");

	const cvMode = getState(config.cv_mode) ?? "unavailable";
	const vvMode = getState(config.vv_mode) ?? "unavailable";
	const cvNight = config.cv_night ? getState(config.cv_night) : null;
	const vvSchedule = config.vv_schedule ? getState(config.vv_schedule) : null;
	const auxHeating = config.aux_heating ? getState(config.aux_heating) : null;

	// heating element sensor used when aux select = "Automatic"
	const heatingElementState = getStateByName("Heating element");

	// circulation pump sensor used to show/hide CV pump & CV flow gifs
	const circPumpSensorState = getStateByName("Circ. pump CV");
	
	// geothermal/brine pump sensor used for BW pumps (to show brine loop & cold side temps)
	const geothermalPumpState = getStateByName("Circ. pump geothermal");
    
	// Build aux icon HTML according to rules:
	// - "Off" -> don't show
	// - "Automatic" -> show aux_on.gif when heating active, aux_auto.gif when inactive
	// - "Backup operation" -> always show aux_on.gif (always on mode)
	let auxDiagramIconHtml = "";
	let auxChipIconHtml = "";
	if (auxHeating) {
		const auxNorm = String(auxHeating).toLowerCase();
		if (auxNorm === "off") {
			// both empty -> nothing shown
			auxDiagramIconHtml = "";
			auxChipIconHtml = "";
		} else {
			// compute chip icon color based on mode
			let chipIconColor;
			if (auxNorm === "backup operation") {
				// Backup operation mode (always on) - yellow icon and aux_on.gif
				chipIconColor = "var(--warning-color, #fdd835)";
				auxDiagramIconHtml = `<img
					src="${imageBase}aux_on.gif"
					class="diagram-icon icon-aux-gif"
					data-icon-key="aux"
					alt="Aux On" />`;
			} else if (auxNorm === "automatic") {
				const heatingActive = heatingElementState && (heatingElementState.toLowerCase() === "on");
				// Use same color scheme as CV/VV chips for automatic mode
				chipIconColor = "var(--chip-active-text, var(--primary-text-color))";
				// Show aux_on.gif when heating is active, aux_auto.gif when inactive
				if (heatingActive) {
					auxDiagramIconHtml = `<img
						src="${imageBase}aux_on.gif"
						class="diagram-icon icon-aux-gif"
						data-icon-key="aux"
						alt="Aux On" />`;
				} else {
					auxDiagramIconHtml = `<img
						src="${imageBase}aux_auto.gif"
						class="diagram-icon icon-aux-gif"
						data-icon-key="aux"
						alt="Aux Auto" />`;
				}
			} else {
				// fallback keep disabled color
				chipIconColor = "var(--disabled-text-color)";
			}

			// chip icon: do NOT include diagram-element; give a chip-specific class so it flows inside the chip
			auxChipIconHtml = `<ha-icon
				class="icon-aux chip-icon"
				style="color:${chipIconColor};"
				icon="mdi:lightning-bolt-outline">
			</ha-icon>`;
		}
	}

	const outdoor = config.outdoor_temp ? getState(config.outdoor_temp) : null;
	const curveTemp = config.curve_temp ? getState(config.curve_temp) : null;
	const tankCv = config.storage_tank_cv ? getState(config.storage_tank_cv) : null;
	const tankVv = config.storage_tank_vv ? getState(config.storage_tank_vv) : null;
	const power = config.em23_power ? getState(config.em23_power) : null;

	const evapTemp = config.evaporator_temp ? getState(config.evaporator_temp) : null;
	// For BW testing: if config not set, try common entity IDs
	const coldSideWarmTemp = config.cold_side_warm_temp ? getState(config.cold_side_warm_temp) : 
		(isPumpTypeBW ? getState("sensor.cold_side_warm_temp") : null);
	const coldSideColdTemp = config.cold_side_cold_temp ? getState(config.cold_side_cold_temp) : 
		(isPumpTypeBW ? getState("sensor.cold_side_cold_temp") : null);
	const hpTemp = config.hp_temp ? getState(config.hp_temp) : null;
	const lpTemp = config.lp_temp ? getState(config.lp_temp) : null;
	const cvForwardTemp = config.cv_forward_temp ? getState(config.cv_forward_temp) : null;
	const cvReturnTemp = config.cv_return_temp ? getState(config.cv_return_temp) : null;

	const compState = config.comp_icon ? getState(config.comp_icon) : null;
	const cvPumpState = config.cv_pump_icon ? getState(config.cv_pump_icon) : null;
	const defrostState = config.defrost_icon ? getState(config.defrost_icon) : null;

	const cvActive = isModeActive(cvMode);
	const vvActive = isModeActive(vvMode, vvSchedule);
	const auxActive = isModeActive(auxHeating);

	const cvIconColor = cvActive
			? "var(--state-climate-heat-color, var(--accent-color))"
			: "var(--disabled-text-color)";

	const vvIconColor = vvActive
			? "var(--state-water-heater-heat-color, var(--accent-color))"
			: "var(--disabled-text-color)";

	const auxIconColor = auxActive
			? "var(--warning-color, #fdd835)"
			: "var(--disabled-text-color)";

	const vvScheduleColor = vvActive
			? "var(--state-water-heater-heat-color, var(--accent-color))"
			: "var(--disabled-text-color)";

	const infoEntities = Array.isArray(config.info_entities) ? config.info_entities : [];
	const cvEntities = Array.isArray(config.cv_entities) ? config.cv_entities : [];
	const vvEntities = Array.isArray(config.vv_entities) ? config.vv_entities : [];
	const auxEntities = Array.isArray(config.aux_entities) ? config.aux_entities : [];

	const iconEntityMap = {
		defrost: config.defrost_icon,
		comp: config.comp_icon,
		cvPump: config.cv_pump_icon,
		aux: config.heating_element || config.aux_heating,
	};

	const cvNightIcon = cvNight ? CV_NIGHT_ICONS[cvNight] ?? null : null;
	const vvScheduleIcon = vvSchedule ? VV_SCHEDULE_ICONS[vvSchedule] ?? null : null;

	const heatCurveChipHtml = `
    <div class="mode-bar mode-bar--bottom">
      <div class="mode-chip heat-curve-trigger mode-chip--info mode-chip--active clickable" data-heat-curve-trigger="true">
        <ha-icon icon="mdi:chart-bell-curve-cumulative"></ha-icon>
        <span class="chip-label">CV Curve</span>
        ${curveTemp !== null ? `<span class="chip-value">${valueWithUnit("curve", curveTemp)}</span>` : ""}
      </div>
    </div>
  `;

	const infoChipClasses = "mode-chip popup-chip mode-chip--info mode-chip--active";
	const cvChipClasses = `mode-chip popup-chip ${chipStateClass(cvActive)}`;
	const vvChipClasses = `mode-chip popup-chip ${chipStateClass(vvActive)}`;
	const auxChipClasses = `mode-chip popup-chip ${chipStateClass(auxActive)}`;

	const html = `
    <img src="${imageBase}/${baseDiagram}" class="diagram-base" alt="${pumpType || 'AW'} diagram" />

    ${
			outdoor !== null
				? `<div class="diagram-label label-outdoor" data-key="outdoor">${valueWithUnit(
						"outdoor",
						outdoor,
				  )}</div>`
				: ""
		}

    ${"" /* heat-curve label moved into bottom chip */}

    ${
			!isPumpTypeBW && evapTemp !== null && compState === "on"
				? `<div class="diagram-label label-evaporator" data-key="evap">${valueWithUnit(
						"evap",
						evapTemp,
				  )}</div>`
				: ""
		}

    ${
			isPumpTypeBW && coldSideWarmTemp !== null && geothermalPumpState === "on"
				? `<div class="diagram-label label-cold-side-warm" data-key="coldSideWarm">${valueWithUnit(
						"coldSideWarm",
						coldSideWarmTemp,
				  )}</div>`
				: ""
		}

    ${
			isPumpTypeBW && coldSideColdTemp !== null && geothermalPumpState === "on"
				? `<div class="diagram-label label-cold-side-cold" data-key="coldSideCold">${valueWithUnit(
						"coldSideCold",
						coldSideColdTemp,
				  )}</div>`
				: ""
		}

    ${
			hpTemp !== null && compState === "on"
				? `<div class="diagram-label label-hp" data-key="hp">${valueWithUnit("hp", hpTemp)}</div>`
				: ""
		}

    ${
			lpTemp !== null && compState === "on"
				? `<div class="diagram-label label-lp" data-key="lp">${valueWithUnit("lp", lpTemp)}</div>`
				: ""
		}

    ${
			tankCv !== null
				? `<div class="diagram-label label-tank-cv" data-key="tankCv">${valueWithUnit("tankCv", tankCv)}</div>`
				: ""
		}

    ${
			tankVv !== null
				? (vvActive
						? `<div class="diagram-label label-tank-vv" data-key="tankVv">${valueWithUnit(
								"tankVv",
								tankVv,
						  )}</div>`
						: "")
				: ""
		}

    ${
			cvForwardTemp !== null
				? `<div class="diagram-label label-cv-forward" data-key="cvForward">${valueWithUnit(
						"cvForward",
						cvForwardTemp,
				  )}</div>`
				: ""
		}

    ${
			cvReturnTemp !== null
				? `<div class="diagram-label label-cv-return" data-key="cvReturn">${valueWithUnit(
						"cvReturn",
						cvReturnTemp,
				  )}</div>`
				: ""
		}

    <div class="diagram-icon icon-cv-pump" data-icon-key="cvPump" style="opacity:${/* gated by circ pump sensor */ (circPumpSensorState === "on" ? opacityFor(cvPumpState) : 0)};">
      <img src="${imageBase}CV_on.gif" alt="CV pump" />
    </div>
    <div class="diagram-icon icon-cv-flow" data-icon-key="cvPump" style="opacity:${(circPumpSensorState === "on" ? opacityFor(cvPumpState) : 0)};">
      <img src="${imageBase}CVflow_on.gif" alt="CV flow" />
    </div>

    <div class="diagram-icon icon-hp-loop" data-icon-key="comp" data-pump-type="${isPumpTypeBW ? 'bw' : 'aw'}" style="opacity:${isPumpTypeBW ? (geothermalPumpState === "on" ? 1 : 0) : (compState === "on" ? 1 : 0)};">
      <img src="${imageBase}/${evapLoopImage}" alt="${isPumpTypeBW ? 'Brine' : 'HP'} on" />
    </div>
    <div class="diagram-icon icon-comp-unit" data-icon-key="comp" data-pump-type="${isPumpTypeBW ? 'bw' : 'aw'}" style="opacity:${compState === "on" ? 1 : 0};">
      <img src="${imageBase}/${compressorImage}" alt="Compressor on" />
    </div>

    ${
			!isPumpTypeBW && defrostState !== null
				? `<ha-icon
             class="diagram-element icon-defrost"
             data-icon-key="defrost"
             style="color:${defrostState === "on" ? "orange" : "var(--disabled-text-color)"};"
             icon="mdi:snowflake-melt">
           </ha-icon>`
				: ""
		}

    ${auxDiagramIconHtml}

    <div class="mode-bar">
      ${
				infoEntities.length
					? `<div class="${infoChipClasses}" data-popup="info">
               <ha-icon icon="mdi:information-slab-circle"></ha-icon>
               <span class="chip-label">Info</span>
               ${power !== null ? `<span class="chip-value">${power} kW</span>` : ""}
             </div>`
					: ""
			}

      ${
				cvEntities.length
					? `<div class="${cvChipClasses}" data-popup="cv">
               <ha-icon icon="mdi:radiator" style="color:${cvIconColor};"></ha-icon>
               ${
									cvNightIcon
										? `<ha-icon class="small-mode-icon" icon="${cvNightIcon}" style="color:${cvIconColor};"></ha-icon>`
										: ""
								}
               <span class="chip-label">CV</span>
             </div>`
					: ""
			}

      ${
				vvEntities.length
					? `<div class="${vvChipClasses}" data-popup="vv">
               <ha-icon icon="mdi:shower-head" style="color:${vvIconColor};"></ha-icon>
               ${
									vvScheduleIcon
										? `<ha-icon class="small-mode-icon" icon="${vvScheduleIcon}" style="color:${vvScheduleColor};"></ha-icon>`
										: ""
								}
               <span class="chip-label">VV</span>
             </div>`
					: ""
			}

      ${
				auxEntities.length
					? `<div class="${auxChipClasses}" data-popup="aux">
               ${auxChipIconHtml ? auxChipIconHtml : `<ha-icon icon="mdi:lightning-bolt-outline" style="color:${auxIconColor};"></ha-icon>`}
                <span class="chip-label">AUX</span>
              </div>`
					: ""
			}
    </div>
    ${heatCurveChipHtml}
  `;

	return {
		html,
		stateEntityMap,
		iconEntityMap,
		chipGroups: {
			info: { title: CHIP_TITLES.info, entities: infoEntities },
			cv: { title: CHIP_TITLES.cv, entities: cvEntities },
			vv: { title: CHIP_TITLES.vv, entities: vvEntities },
			aux: { title: CHIP_TITLES.aux, entities: auxEntities },
		},
	};
}
