export class LvHeatpumpCardEditor extends HTMLElement {
	constructor() {
		super();
		this._config = {};
		this._hass = null;
		this._formBasic = null;
		this._formAdvanced = null;
		this._basicSchema = [];
		this._advancedSchema = [];
	}

	setConfig(config) {
		this._config = config || {};
		this._buildSchema();
		this._render();
	}

	set hass(hass) {
		this._hass = hass;
		if (this._formBasic) this._formBasic.hass = hass;
		if (this._formAdvanced) this._formAdvanced.hass = hass;
		if (!this.childElementCount) this._render();
	}

	_buildSchema() {
		this._basicSchema = [
			{
				name: "device_id",
				label: "DVI X MQTT device",
				selector: { device: { integration: "mqtt" } },
			},
			{
				name: "show_temp_unit",
				label: "Show temperature unit (°C/°F)",
				selector: { boolean: {} },
			},
		];

		this._advancedSchema = [
			{ name: "cv_mode", label: "Central heating mode (cv_mode)", selector: { entity: { domain: "select" } } },
			{ name: "vv_mode", label: "Hot water mode (vv_mode)", selector: { entity: { domain: "select" } } },
			{ name: "cv_night", label: "CV night mode (cv_night)", selector: { entity: { domain: "select" } } },
			{ name: "vv_schedule", label: "VV schedule (vv_schedule)", selector: { entity: { domain: "select" } } },
			{ name: "aux_heating", label: "Aux / electric heater mode (aux_heating)", selector: { entity: { domain: "select" } } },
			{ name: "heating_config", label: "Heating configuration (heating_config)", selector: { entity: { domain: "select" } } },			
			{ name: "vv_setpoint", label: "Warm water setpoint (vv_setpoint)", selector: { entity: { domain: ["number", "input_number"] } } },
			{ name: "cv_curve_number", label: "Curve shift (cv_curve)", selector: { entity: { domain: "number" } } },
			{ name: "curve_set_minus12_number", label: "Curve temp at -12°C (curve_set_12)", selector: { entity: { domain: "number" } } },
			{ name: "curve_set_plus12_number", label: "Curve temp at +12°C (curve_set_12_2)", selector: { entity: { domain: "number" } } },
			{ name: "cv_min_number", label: "CV min temperature (cv_min)", selector: { entity: { domain: "number" } } },
			{ name: "cv_max_number", label: "CV max temperature (cv_max)", selector: { entity: { domain: "number" } } },
			{ name: "outdoor_temp", label: "Outdoor temperature (outdoor_temp)", selector: { entity: { domain: "sensor" } } },
			{ name: "curve_temp", label: "Curve temperature (curve_temp)", selector: { entity: { domain: "sensor" } } },
			{ name: "storage_tank_cv", label: "Storage tank CV (storage_tank_cv)", selector: { entity: { domain: "sensor" } } },
			{ name: "storage_tank_vv", label: "Storage tank VV (storage_tank_vv)", selector: { entity: { domain: "sensor" } } },
			{ name: "evaporator_temp", label: "Evaporator temperature (evaporator_temp)", selector: { entity: { domain: "sensor" } } },
			{ name: "hp_temp", label: "High pressure temperature (hp_temp)", selector: { entity: { domain: "sensor" } } },
			{ name: "lp_temp", label: "Low pressure temperature (lp_temp)", selector: { entity: { domain: "sensor" } } },
			{ name: "cv_forward_temp", label: "CV forward temperature (cv_forward_temp)", selector: { entity: { domain: "sensor" } } },
			{ name: "cv_return_temp", label: "CV return temperature (cv_return_temp)", selector: { entity: { domain: "sensor" } } },
			{ name: "em23_power", label: "EM23 power (em23_power)", selector: { entity: { domain: "sensor" } } },
			{ name: "em23_energy", label: "EM23 energy (em23_energy)", selector: { entity: { domain: "sensor" } } },
			{ name: "install_date", label: "Installation date sensor", selector: { entity: { domain: "sensor" } } },
            { name: "service_date", label: "Service date sensor", selector: { entity: { domain: "sensor" } } },
			{ name: "comp_icon", label: "Compressor state (comp_icon)", selector: { entity: { domain: "binary_sensor" } } },
			{ name: "cv_pump_icon", label: "CV pump state (cv_pump_icon)", selector: { entity: { domain: "binary_sensor" } } },
			{ name: "defrost_icon", label: "Defrost state (defrost_icon)", selector: { entity: { domain: "binary_sensor" } } },
		];
	}

	_render() {
		if (!this._hass) return;

		this.innerHTML = "";

		const container = document.createElement("div");
		container.className = "card-config";

		const basicForm = document.createElement("ha-form");
		basicForm.schema = this._basicSchema;
		basicForm.data = this._config;
		basicForm.hass = this._hass;
		basicForm.addEventListener("value-changed", (ev) => {
			const oldDevice = this._config.device_id;
			this._config = { ...this._config, ...ev.detail.value };
			const newDevice = this._config.device_id;
			if (newDevice && newDevice !== oldDevice) {
				this._autoFillFromDevice(newDevice);
			}
			this._dispatchConfigChanged();
		});
		container.appendChild(basicForm);
		this._formBasic = basicForm;

		const details = document.createElement("details");
		details.className = "advanced-config";

		const summary = document.createElement("summary");
		summary.className = "advanced-config__summary";
		summary.textContent = "Advanced entity mapping";
		summary.style.cursor = "pointer";
		details.appendChild(summary);

		const advWrapper = document.createElement("div");
		advWrapper.className = "advanced-config__body";

		const advForm = document.createElement("ha-form");
		advForm.schema = this._advancedSchema;
		advForm.data = this._config;
		advForm.hass = this._hass;
		advForm.addEventListener("value-changed", (ev) => {
			this._config = { ...this._config, ...ev.detail.value };
			this._dispatchConfigChanged();
		});

		advWrapper.appendChild(advForm);
		this._formAdvanced = advForm;

		const resetBtn = document.createElement("button");
		resetBtn.type = "button";
		resetBtn.className = "advanced-config__btn";
		resetBtn.textContent = "Auto-fill from device";
		resetBtn.addEventListener("click", () => {
			if (!this._config.device_id) return;
			this._autoFillFromDevice(this._config.device_id);
			this._dispatchConfigChanged();
		});
		advWrapper.appendChild(resetBtn);

		details.appendChild(advWrapper);
		container.appendChild(details);

		this.appendChild(container);
	}

	_autoFillFromDevice(deviceId) {
		if (!this._hass || !this._hass.entities) return;

		const entityIds = Object.keys(this._hass.states).filter((eid) => {
			const reg = this._hass.entities[eid];
			return reg && reg.device_id === deviceId;
		});

		const find = (domain, suffix) =>
			entityIds.find(
				(eid) =>
					eid.startsWith(`${domain}.`) &&
					(suffix.startsWith("_") ? eid.endsWith(suffix) : eid.endsWith(`_${suffix}`)),
			);

		const patch = {
			cv_mode: find("select", "cv_mode"),
			vv_mode: find("select", "vv_mode"),
			cv_night: find("select", "cv_night"),
			vv_schedule: find("select", "vv_schedule"),
			aux_heating: find("select", "aux_heating"),
			heating_config: find("select", "heating_config"),
			vv_setpoint: find("number", "vv_setpoint") || find("input_number", "vv_setpoint"),
			cv_curve_number: find("number", "cv_curve"),
			curve_set_minus12_number: find("number", "curve_set_12"),
			curve_set_plus12_number: find("number", "curve_set_12_2"),
			cv_min_number: find("number", "cv_min"),
			cv_max_number: find("number", "cv_max"),
			outdoor_temp: find("sensor", "outdoor"),
			curve_temp: find("sensor", "curve_temp"),
			storage_tank_cv: find("sensor", "storage_tank_cv"),
			storage_tank_vv: find("sensor", "storage_tank_vv"),
			evaporator_temp: find("sensor", "evaporator"),
			hp_temp: find("sensor", "compressor_hp"),
			lp_temp: find("sensor", "compressor_lp"),
			cv_forward_temp: find("sensor", "cv_forward"),
			cv_return_temp: find("sensor", "cv_return"),
			em23_power: find("sensor", "em23_power"),
			em23_energy: find("sensor", "em23_energy"),
			install_date: find("sensor", "install_date"),
            service_date: find("sensor", "service_date"),
			comp_hours: find("sensor", "comp_hours"),
			vv_hours: find("sensor", "vv_hours"),
			heating_hours: find("sensor", "heating_hours"),
			comp_icon: find("binary_sensor", "soft_starter_compressor"),
			cv_pump_icon: find("binary_sensor", "circ_pump_cv"),
			defrost_icon: find("binary_sensor", "4_way_valve_defrost"),
			
		};

		const info_entities = [
			patch.em23_energy,
			patch.em23_power,
			patch.install_date,
            patch.service_date,
			patch.comp_hours,
			patch.vv_hours,
			patch.heating_hours,		


		].filter(Boolean);

		const cv_entities = [
			patch.cv_mode,
			patch.cv_curve_number,			
			patch.heating_config,
			patch.cv_night,
		].filter(Boolean);

		const vv_entities = [patch.vv_setpoint, patch.vv_mode, patch.vv_schedule].filter(Boolean);

		const aux_entities = [
			patch.aux_heating,
			patch.heating_hours,
		].filter(Boolean);

		this._config = {
			...this._config,
			...patch,
			info_entities,
			cv_entities,
			vv_entities,
			aux_entities,
		};

		this._render();
	}

	_dispatchConfigChanged() {
		this.dispatchEvent(
			new CustomEvent("config-changed", {
				detail: { config: this._config },
				bubbles: true,
				composed: true,
			}),
		);
	}
}

if (!customElements.get("lv-heatpump-card-editor")) {
	customElements.define("lv-heatpump-card-editor", LvHeatpumpCardEditor);
}
