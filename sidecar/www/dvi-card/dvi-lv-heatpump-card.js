import { HEAT_CURVE_DEFAULTS } from "./dvi-lv/heat_curve_card.js";
import { buildDiagramView } from "./dvi-lv/diagram.js";
import { bindHistoryHooks, bindIconHooks } from "./dvi-lv/history-hooks.js";
import { wireModeChips } from "./dvi-lv/mode-chips.js";
import "./dvi-lv/editor.js";

class LvHeatpumpCard extends HTMLElement {
	constructor() {
		super();
		this._heatCurveConfigKey = "";
	}

	static get imageBase() {
		return new URL("./dvi-lv/", import.meta.url).href;
	}

	setConfig(config) {
		if (!config.cv_mode || !config.vv_mode) {
			throw new Error("You must define at least 'cv_mode' and 'vv_mode' entities");
		}
		this._config = config;
		this._root = this.attachShadow({ mode: "open" });
		const styleUrl = new URL("./dvi-lv/heatpump-card.css", import.meta.url).href;
		this._root.innerHTML = `
      <style>@import url("${styleUrl}");</style>
      <ha-card>
        <div class="header">DVI LV Compact varmepumpe</div>
        <div class="diagram" id="diagram"></div>
      </ha-card>
    `;
	}

	set hass(hass) {
		this._hass = hass;
		if (!this._config || !this._root) return;

		const diagram = this._root.getElementById("diagram");
		if (!diagram) return;

		const view = buildDiagramView({
			hass,
			config: this._config,
			imageBase: LvHeatpumpCard.imageBase,
		});

		diagram.innerHTML = view.html;
		bindHistoryHooks(diagram, view.stateEntityMap, this);
		bindIconHooks(diagram, view.iconEntityMap, this);
		wireModeChips(diagram, hass, view.chipGroups);
		this._bindHeatCurveTrigger(diagram);
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
		return document.createElement("lv-heatpump-card-editor");
	}

	static getStubConfig() {
		return { show_temp_unit: false };
	}
}

window.customCards = window.customCards || [];
window.customCards.push({
	type: "lv-heatpump-card",
	name: "DVI LV Heatpump Card",
	description: "Visual overview and control panel for a DVI LV heatpump.",
	preview: true,
	documentationURL: "https://github.com/ruteclrp/dvi-bridge-standalone",
});

if (!customElements.get("lv-heatpump-card")) {
	customElements.define("lv-heatpump-card", LvHeatpumpCard);
}
