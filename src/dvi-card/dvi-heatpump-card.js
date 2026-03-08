import { HEAT_CURVE_DEFAULTS } from "./dvi/heat_curve_card.js";
import { buildDiagramView } from "./dvi/diagram.js";
import { bindHistoryHooks, bindIconHooks } from "./dvi/history-hooks.js";
import { wireModeChips } from "./dvi/mode-chips.js";
import "./dvi/editor.js";

class DviHeatpumpCard extends HTMLElement {
	constructor() {
		super();
		this._heatCurveConfigKey = "";
		this._pumpType = "AW"; // Default to AW
		this._alarmLatched = false;
		this._alarmWasActive = false;
	}

	async _fetchPumpType() {
		// Try to get pump type from Home Assistant entity first
		if (this._hass && this._config && this._config.pump_type) {
			const pumpTypeEntity = this._hass.states[this._config.pump_type];
			if (pumpTypeEntity && pumpTypeEntity.state) {
				this._pumpType = pumpTypeEntity.state;
				this._updateHeader();
				if (this._root) {
					this._renderDiagram();
				}
				return;
			}
		}

		// Fallback to /api/pump_type for standalone mode
		try {
			const response = await fetch("/api/pump_type");
			const data = await response.json();
			if (data && data.pump_type) {
				this._pumpType = data.pump_type;
				this._updateHeader();
				if (this._hass && this._config && this._root) {
					this._renderDiagram();
				}
			}
		} catch (e) {
			console.warn("Could not fetch pump type, using default AW:", e);
		}
	}

	static get imageBase() {
		return new URL("./dvi/", import.meta.url).href;
	}

	_updateHeader() {
		if (!this._root) return;
		const header = this._root.getElementById("card-header");
		if (header) {
			const pumpTypeLabel = this._pumpType === "BW" ? "BW" : "AW";
			header.textContent = `DVI ${pumpTypeLabel} Compact varmepumpe`;
		}
	}

	setConfig(config) {
		if (!config.cv_mode || !config.vv_mode) {
			throw new Error("You must define at least 'cv_mode' and 'vv_mode' entities");
		}
		this._config = config;
		this._root = this.attachShadow({ mode: "open" });
		const styleUrl = new URL("./dvi/heatpump-card.css", import.meta.url).href;
		const styleUrlWithCacheBust = `${styleUrl}?v=${Date.now()}`;
		this._root.innerHTML = `
      <style>@import url("${styleUrlWithCacheBust}");</style>
      <ha-card>
        <div class="header" id="card-header"></div>
        <div class="mode-chips-bar" id="mode-chips-bar"></div>
        <div class="diagram" id="diagram"></div>
				<div class="alarm-overlay" id="alarm-overlay" aria-hidden="true">
					<div class="alarm-overlay__popup" role="alert" aria-live="assertive">
						<div class="alarm-overlay__text">ALARM</div>
						<button class="alarm-overlay__ack" id="alarm-ack" type="button">Acknowledge</button>
					</div>
				</div>
      </ha-card>
    `;
		this._bindAlarmControls();
		this._updateHeader();
		this._fetchPumpType(); // Try to fetch pump type after config is set
	}

	set hass(hass) {
		this._hass = hass;
		// Check for pump type updates from HA entity
		if (this._config && this._config.pump_type) {
			const pumpTypeEntity = hass.states[this._config.pump_type];
			if (pumpTypeEntity && pumpTypeEntity.state !== this._pumpType) {
				this._pumpType = pumpTypeEntity.state;
				this._updateHeader();
			}
		}
		this._renderDiagram();
	}

	_renderDiagram() {
		if (!this._config || !this._root || !this._hass) return;

		const diagram = this._root.getElementById("diagram");
		const modeChipsBar = this._root.getElementById("mode-chips-bar");
		if (!diagram || !modeChipsBar) return;

		const view = buildDiagramView({
			hass: this._hass,
			config: this._config,
			imageBase: DviHeatpumpCard.imageBase,
			pumpType: this._pumpType,
		});

		diagram.innerHTML = view.diagramHtml;
		modeChipsBar.innerHTML = view.chipsHtml;
		bindHistoryHooks(diagram, view.stateEntityMap, this);
		bindIconHooks(diagram, view.iconEntityMap, this);
		wireModeChips(modeChipsBar, this._hass, view.chipGroups);
		this._bindHeatCurveTrigger(modeChipsBar);
		this._renderAlarmOverlay();
	}

	_renderAlarmOverlay() {
		const overlay = this._root?.getElementById("alarm-overlay");
		if (!overlay) return;

		const active = this._isSumAlarmActive();
		if (active && !this._alarmWasActive) {
			this._alarmLatched = true;
		}

		this._alarmWasActive = active;
		this._setAlarmOverlayVisible(this._alarmLatched);
	}

	_setAlarmOverlayVisible(visible) {
		const overlay = this._root?.getElementById("alarm-overlay");
		if (!overlay) return;

		overlay.classList.toggle("alarm-overlay--active", visible);
		overlay.setAttribute("aria-hidden", visible ? "false" : "true");
	}

	_bindAlarmControls() {
		const acknowledgeButton = this._root?.getElementById("alarm-ack");
		if (!acknowledgeButton) return;

		acknowledgeButton.onclick = async () => {
			this._alarmLatched = false;
			this._setAlarmOverlayVisible(false);

			if (!this._hass?.connection) {
				try {
					await fetch("/api/alarm/sumalarm/ack", { method: "POST" });
					if (this._hass?.refresh) {
						await this._hass.refresh();
						this.hass = this._hass;
					}
				} catch (error) {
					console.warn("Could not acknowledge sidecar alarm latch:", error);
				}
			}
		};
	}

	_isSumAlarmActive() {
		const entityId = this._resolveSumAlarmEntityId();
		if (!entityId || !this._hass?.states?.[entityId]) return false;

		const state = String(this._hass.states[entityId].state).toLowerCase();
		return state === "on" || state === "1" || state === "true" || state === "alarm";
	}

	_resolveSumAlarmEntityId() {
		const configuredEntityId = this._config?.sumalarm || this._config?.sum_alarm_failure;
		if (configuredEntityId && this._hass?.states?.[configuredEntityId]) {
			return configuredEntityId;
		}

		const stateIds = Object.keys(this._hass?.states || {});
		return (
			stateIds.find((entityId) => entityId.endsWith(".sum_alarm_failure")) ||
			stateIds.find((entityId) => entityId.endsWith(".sumalarm")) ||
			null
		);
	}

	_bindHeatCurveTrigger(diagram) {
		const trigger = diagram.querySelector("[data-heat-curve-trigger]");
		if (!trigger) return;
		trigger.onclick = () => this._openHeatCurvePopup();
	}

	_openHeatCurvePopup() {
		if (!this._hass) return;
		this._hass.callService("browser_mod", "popup", {
			title: "Kurvetemperatur",
			size: "wide",
			content: {
				type: "custom:heat-curve-card",
				title: "Kurvetemperatur",
				entities: this._getHeatCurveEntities(),
			},
		});
	}

	_getHeatCurveEntities() {
		const cfg = this._config || {};
		const overrides = {
			cv_curve: cfg.cv_curve_number,
			curve_set_minus12: cfg.curve_set_minus12_number,
			curve_set_plus12: cfg.curve_set_plus12_number,
			cv_min: cfg.cv_min_number,
			cv_max: cfg.cv_max_number,
			outdoor: cfg.outdoor_temp,
			curve_sensor: cfg.curve_temp,
		};
		const merged = { ...HEAT_CURVE_DEFAULTS };
		Object.entries(overrides).forEach(([key, value]) => {
			if (value) merged[key] = value;
		});
		return merged;
	}

	static getConfigElement() {
		return document.createElement("dvi-heatpump-card-editor");
	}

	static getStubConfig() {
		return { show_temp_unit: false };
	}
}

window.customCards = window.customCards || [];
window.customCards.push({
	type: "dvi-heatpump-card",
	name: "DVI Heatpump Card",
	description: "Visual overview and control panel for a DVI heatpump.",
	preview: true,
	documentationURL: "https://github.com/ruteclrp/dvi-bridge-standalone",
});

if (!customElements.get("dvi-heatpump-card")) {
	customElements.define("dvi-heatpump-card", DviHeatpumpCard);
}
