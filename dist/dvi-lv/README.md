# DVI LV Frontend Modules

Everything powering the `lv-heatpump-card` lives in this folder. We keep a **single** Lovelace card entry (`../dvi-lv-heatpump-card.js`) so users configure entity IDs once, while UI logic is split across focused ES modules.

```
dist/
├─ dvi-lv-heatpump-card.js         ← Lovelace card entry (imports modules below)
└─ dvi-lv/
   ├─ heatpump-card.css            ← shared Shadow DOM styles
   ├─ diagram.js                   ← diagram layout + chip metadata
   ├─ history-hooks.js             ← more-info/history listeners
   ├─ mode-chips.js                ← browser_mod popup wiring
   ├─ heat_curve_card.js           ← reusable heat-curve card
   ├─ editor.js                    ← card editor & auto-fill logic
   └─ README.md                    ← this guide
```

## Why this split?

* **Single source of truth** – one editor, one config object, no duplicate entity setup.
* **Parallel development** – contributors can modify isolated modules instead of a monolithic 800+ line file.
* **Reusability** – helpers (`buildDiagramView`, `wireModeChips`, `heat-curve-card`) can be reused by future cards/add-ons.

## Runtime lifecycle

1. `LvHeatpumpCard.setConfig` validates required entities and renders the base DOM.
2. `set hass` gathers states, calls `buildDiagramView` for HTML + metadata, injects it into `.diagram`.
3. `bindHistoryHooks` / `bindIconHooks` attach native `hass-more-info`.
4. `wireModeChips` converts chip clicks into `browser_mod.popup`.
5. `heat_curve_card` is instantiated inside popups when “Kurvetemperatur” is clicked.
6. `editor.js` exposes the Lovelace editor schema and device auto-fill.

## Adding a new visual section

1. Create a module in `dvi-lv/` that exports a builder.
2. Import it inside `dvi-lv-heatpump-card.js`.
3. Insert its HTML and (optionally) provide metadata/hooks.

```javascript
// filepath: c:\Dev\DVI\dvi-bridge-standalone\dist\dvi-lv\custom-section.js
export function buildCustomSection({ hass, config }) {
	const readState = (entityId) =>
		entityId && hass.states[entityId] ? hass.states[entityId].state : null;

	const sensor = readState(config.extra_sensor) ?? "unavailable";

	return {
		html: `
      <div class="custom-section">
        <span>Extra sensor:</span>
        <strong>${sensor}</strong>
      </div>
    `,
	};
}
```

Use it:

```javascript
// ...existing imports...
import { buildCustomSection } from "./dvi-lv/custom-section.js";

// ...inside set hass() after diagram HTML is set...
const section = buildCustomSection({ hass, config: this._config });
diagram.insertAdjacentHTML("beforeend", section.html);
```

Add CSS either inline or by extending the `<style>` block in `dvi-lv-heatpump-card.js`.

## Styling guidance
* `heatpump-card.css` holds every diagram/chip rule. Extend it (using the existing section comments) instead of adding inline styles.
* `heat_curve_card.css` styles the popup chart/control layout; both embedded and standalone cards import this file, so keep shared rules here.
* Prefer semantic classes or `data-*` hooks (e.g., `heat-curve-trigger`, `heat-curve-card__controls`) rather than generic IDs like `controls`.
* Dynamic, state-dependent values (opacity, colors) may still be set via JS, but static positioning/spacing should live in CSS.

## Reading DVI entities

Keep helpers close to the modules:

```javascript
const readState = (entityId) =>
	entityId && hass.states[entityId] ? hass.states[entityId].state : null;

const readUnit = (entityId) =>
	entityId && hass.states[entityId]
		? hass.states[entityId].attributes.unit_of_measurement || ""
		: "";
```

Example usage:

```javascript
const curveTemp = readState(config.curve_temp);
const unit = readUnit(config.curve_temp);
const formatted = curveTemp !== null ? `${curveTemp}${unit ? ` ${unit}` : ""}` : "n/a";
```

## Using external sensors

Expose selectors in `editor.js`:

```javascript
this._advancedSchema.push({
	name: "weather_station_temp",
	label: "External outdoor sensor",
	selector: { entity: { domain: "sensor" } },
});
```

Consume them:

```javascript
const outdoor =
	readState(config.weather_station_temp) ?? readState(config.outdoor_temp);
```

## Extending the editor & auto-fill

1. Add schema entries (basic or advanced).
2. Update `_autoFillFromDevice` to guess the new entity suffix.
3. Re-render so the forms refresh.

```javascript
const patch = {
	// ...existing mappings...
	extra_sensor: find("sensor", "extra_sensor"),
};

this._config = {
	...this._config,
	...patch,
};
this._render();
```

## Popups with browser_mod

### Chips

`mode-chips.js` handles all chip popups:

```javascript
export function wireModeChips(container, hass, chipGroups = {}) {
	container.querySelectorAll(".popup-chip").forEach((chip) => {
		const group = chipGroups[chip.dataset.popup];
		if (!group?.entities?.length) return;

		chip.onclick = () =>
			hass.callService("browser_mod", "popup", {
				title: group.title,
				content: { type: "entities", entities: group.entities },
			});
	});
}
```

To add a chip, return metadata from `buildDiagramView`:

```javascript
chipGroups.diagnostics = {
	title: "Diagnostics",
	entities: ["binary_sensor.soft_starter", "sensor.weather_station_temp"],
};
```

Include the HTML:

```html
<div class="mode-chip popup-chip" data-popup="diagnostics">
  <ha-icon icon="mdi:tools"></ha-icon>
  <span class="chip-label">Diag</span>
</div>
```

### Heat-curve popup

Clicking the “Kurvetemperatur” label triggers:

```javascript
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
```

Any new popup should follow the same pattern—provide a friendly title and embed either core or custom cards.

## heat_curve_card quickstart

`heat_curve_card.js` registers `<heat-curve-card>`:

```javascript
const card = document.createElement("heat-curve-card");
card.setConfig({
	title: "Custom curve",
	entities: {
		cv_curve: "number.dvi_lv12_cv_curve",
		// ...override defaults...
	},
});
card.hass = hass;
```

It expects entity IDs for:
* `cv_curve`, `curve_set_minus12`, `curve_set_plus12`
* `outdoor`, `curve_sensor`
* `cv_min`, `cv_max` (optional but recommended)

## Testing checklist

* Confirm custom elements exist: `customElements.get("lv-heatpump-card")`.
* Verify `browser_mod.popup` calls succeed (requires Browser Mod installed).
* Switch devices in the editor and ensure auto-fill repopulates fields.
* Validate lint/build by re-bundling if needed.

Following these guidelines keeps every collaborator aligned on why the project is structured this way and how to extend it without duplicating configuration screens or breaking the existing UI.