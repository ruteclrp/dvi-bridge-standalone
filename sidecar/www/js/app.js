import "../dvi-card/dvi-lv-heatpump-card.js";
import "../dvi-card/dvi-lv/heat_curve_card.js";
import "../dvi-card/dvi-lv/history_card.js";
import { HassAdapter } from "../hass-adapter.js";

window.addEventListener("DOMContentLoaded", async () => {
  const hass = new HassAdapter();
  await hass.refresh();

  const card = document.createElement("lv-heatpump-card");
  card.setConfig({
    outdoor_temp: "sensor.outdoor_temp",
    curve_temp: "sensor.curve_temp",
    cv_forward_temp: "sensor.cv_forward_temp",
    cv_return_temp: "sensor.cv_return_temp",
    storage_tank_cv: "sensor.storage_tank_cv_temp",
    storage_tank_vv: "sensor.storage_tank_vv_temp",
    evaporator_temp: "sensor.evaporator_temp",
    hp_temp: "sensor.compressor_hp",
    lp_temp: "sensor.compressor_lp",
    em23_power: "sensor.em23_power",
    em23_energy: "sensor.em23_energy",
    cv_mode: "select.cv_mode",
    vv_mode: "select.vv_mode",
    cv_night: "select.cv_night",
    vv_schedule: "select.vv_schedule",
    aux_heating: "select.aux_heating",
    comp_icon: "binary_sensor.soft_starter_compressor",
    cv_pump_icon: "binary_sensor.circ_pump_cv",
    defrost_icon: "binary_sensor.four_way_valve_defrost",
    cv_curve_number: "number.cv_curve",
    curve_set_minus12_number: "number.curve_set_minus12",
    curve_set_plus12_number: "number.curve_set_plus12",
    cv_min_number: "number.cv_min",
    cv_max_number: "number.cv_max",
    info_entities: [
      "sensor.em23_energy",
      "sensor.em23_power",
      "sensor.comp_hours",
      "sensor.vv_hours",
      "sensor.heating_hours"
    ],
    cv_entities: ["select.cv_mode", "number.cv_curve", "select.cv_night"],
    vv_entities: ["number.vv_setpoint", "select.vv_mode", "select.vv_schedule"],
    aux_entities: ["select.aux_heating"]
  });

  card.hass = hass;
  document.body.appendChild(card);

  setupPopupHandler(hass);
  overrideTempSensorClicks(card, hass);

  const chartInterval = setInterval(() => {
    if (window.Chart && !window.Chart._darkModePatched) {
      applyChartDarkMode();
      clearInterval(chartInterval);
    }
  }, 100);

  setInterval(async () => {
    await hass.refresh();
    card.hass = hass;
  }, 2000);
});

function applyChartDarkMode() {
  if (!window.Chart || window.Chart._darkModePatched) return;

  const OriginalChart = window.Chart;
  window.Chart = function (ctx, config) {
    if (config?.options?.scales) {
      if (config.options.scales.x) {
        config.options.scales.x.grid = config.options.scales.x.grid || {};
        config.options.scales.x.grid.color = "rgba(255,255,255,0.3)";
        config.options.scales.x.grid.borderColor = "rgba(255,255,255,0.5)";
        config.options.scales.x.ticks = config.options.scales.x.ticks || {};
        config.options.scales.x.ticks.color = "#fff";
        if (config.options.scales.x.title) {
          config.options.scales.x.title.color = "#fff";
        }
      }
      if (config.options.scales.y) {
        config.options.scales.y.grid = config.options.scales.y.grid || {};
        config.options.scales.y.grid.color = "rgba(255,255,255,0.3)";
        config.options.scales.y.grid.borderColor = "rgba(255,255,255,0.5)";
        config.options.scales.y.ticks = config.options.scales.y.ticks || {};
        config.options.scales.y.ticks.color = "#fff";
        if (config.options.scales.y.title) {
          config.options.scales.y.title.color = "#fff";
        }
      }
    }
    if (config?.options?.plugins?.legend?.labels) {
      config.options.plugins.legend.labels.color = "#fff";
    }
    if (config?.options?.plugins?.tooltip) {
      config.options.plugins.tooltip.titleColor = "#fff";
      config.options.plugins.tooltip.bodyColor = "#fff";
    }
    return new OriginalChart(ctx, config);
  };
  Object.setPrototypeOf(window.Chart, OriginalChart);
  Object.assign(window.Chart, OriginalChart);
  window.Chart._darkModePatched = true;
  console.log("✅ Chart.js dark mode applied");
}

function overrideTempSensorClicks(card, hass) {
  console.log("🔧 overrideTempSensorClicks called");
  const shadowRoot = card.shadowRoot;
  if (!shadowRoot) {
    console.log("❌ No shadowRoot found");
    return;
  }
  console.log("✅ shadowRoot found");

  const keyToEntityMap = {
    outdoor: { entityId: "sensor.outdoor_temp", historyKey: "outdoor_temp" },
    tankCv: { entityId: "sensor.storage_tank_cv_temp", historyKey: "storage_tank_cv" },
    tankVv: { entityId: "sensor.storage_tank_vv_temp", historyKey: "storage_tank_vv" },
    cvForward: { entityId: "sensor.cv_forward_temp", historyKey: "cv_forward_temp" },
    cvReturn: { entityId: "sensor.cv_return_temp", historyKey: "cv_return_temp" },
    evap: { entityId: "sensor.evaporator_temp", historyKey: "evaporator_temp" },
    hp: { entityId: "sensor.compressor_hp", historyKey: "compressor_hp_temp" },
    lp: { entityId: "sensor.compressor_lp", historyKey: "compressor_lp_temp" }
  };

  function overrideHandlers() {
    const diagram = shadowRoot.querySelector(".diagram");
    if (!diagram) {
      console.log("⏳ Diagram not yet ready");
      return;
    }

    const allLabels = diagram.querySelectorAll(".diagram-label");
    const clickableLabels = diagram.querySelectorAll(".diagram-label.clickable");
    console.log(`🔍 Found ${allLabels.length} total labels, ${clickableLabels.length} clickable`);

    let overriddenCount = 0;
    clickableLabels.forEach(label => {
      const dataKey = label.getAttribute("data-key");
      console.log(`  - Label with data-key: ${dataKey}, clickable: ${label.classList.contains("clickable")}`);

      const mapping = keyToEntityMap[dataKey];
      if (!dataKey || !mapping) {
        console.log("    ⏭️ Skipping (not in temperature sensor list)");
        return;
      }
      if (label._historyOverridden) {
        console.log("    ⏭️ Already overridden");
        return;
      }
      label._historyOverridden = true;

      const oldHandler = label.onclick;
      console.log(`    📝 Old handler: ${oldHandler ? "exists" : "null"}`);

      label.onclick = e => {
        console.log(`🖱️ Click detected on ${dataKey}!`);
        e.preventDefault();
        e.stopPropagation();

        const entityState = hass.states[mapping.entityId];
        const friendlyName = entityState?.attributes?.friendly_name || mapping.entityId;

        hass.popupCallback({
          title: `${friendlyName} - 24 timers historik`,
          content: {
            type: "custom:history-card",
            sensor: mapping.historyKey,
            title: friendlyName
          }
        });
      };

      console.log(`    ✅ Override installed for ${dataKey}`);
      overriddenCount++;
    });

    if (overriddenCount > 0) {
      console.log(`✅ Overrode ${overriddenCount} temperature sensor click handlers`);
    }

    const defrostIcon = diagram.querySelector('[data-icon-key="defrost"]');
    if (defrostIcon && !defrostIcon._defrostOverridden) {
      defrostIcon._defrostOverridden = true;
      defrostIcon.classList.add("clickable");
      defrostIcon.onclick = e => {
        console.log("🖱️ Click detected on defrost icon!");
        e.preventDefault();
        e.stopPropagation();

        hass.popupCallback({
          title: "Afrimning - 24 timers historik",
          content: {
            type: "custom:history-card",
            sensor: "defrost",
            title: "Defrost Activity"
          }
        });
      };
      console.log("✅ Defrost icon click handler installed");
    }
  }

  const observer = new MutationObserver(mutations => {
    console.log(`🔄 MutationObserver triggered (${mutations.length} mutations)`);
    overrideHandlers();
  });

  observer.observe(shadowRoot, {
    childList: true,
    subtree: true
  });

  console.log("👀 MutationObserver installed");
  overrideHandlers();
}

function setupPopupHandler(hass) {
  const modal = document.getElementById("modal-overlay");
  const modalTitle = document.getElementById("modal-title");
  const modalBody = document.getElementById("modal-body");
  const modalClose = document.getElementById("modal-close");

  modalClose.onclick = () => modal.classList.remove("active");
  modal.onclick = e => {
    if (e.target === modal) modal.classList.remove("active");
  };

  hass.popupCallback = data => {
    const { title, content } = data;

    if (content && content.type === "custom:history-card") {
      modalTitle.textContent = title || "Temperature History";

      (async () => {
        if (!window.Chart) {
            await import("https://cdn.jsdelivr.net/npm/chart.js@4.4.1/dist/chart.umd.min.js");
        }
        applyChartDarkMode();

        const historyCard = document.createElement("history-card");
        historyCard.setConfig({
          sensor: content.sensor,
          title: content.title || title
        });
        historyCard.hass = hass;

        modalBody.innerHTML = "";
        modalBody.appendChild(historyCard);
      })();

      modal.classList.add("active");
      return;
    }

    if (content && content.type === "custom:heat-curve-card") {
      modalTitle.textContent = title || "Heat Curve";

      (async () => {
        if (!window.Chart) {
            await import("https://cdn.jsdelivr.net/npm/chart.js@4.4.1/dist/chart.umd.min.js");
        }
        applyChartDarkMode();

        const heatCurveCard = document.createElement("heat-curve-card");
        heatCurveCard.setConfig({
          title: content.title || title,
          entities: content.entities
        });
        heatCurveCard.hass = hass;

        modalBody.innerHTML = "";
        modalBody.appendChild(heatCurveCard);
      })();

      modal.classList.add("active");
      return;
    }

    if (!content || !content.entities) return;

    modalTitle.textContent = title || "Details";
    const entities = Array.isArray(content.entities) ? content.entities : Object.values(content.entities);

    modalBody.innerHTML = entities
      .map(entityId => {
        const state = hass.states[entityId];
        if (!state) return "";

        const name = state.attributes?.friendly_name || entityId;
        const stateValue = state.state;
        const unit = state.attributes?.unit_of_measurement || "";
        const domain = entityId.split(".")[0];

        if (domain === "select" && state.attributes?.options) {
          const options = state.attributes.options
            .map(opt => `<option value="${opt}" ${opt === stateValue ? "selected" : ""}>${opt}</option>`)
            .join("");
          return `
            <div class="entity-row">
              <div class="entity-name">${name}</div>
              <select class="entity-control" data-entity="${entityId}" data-domain="${domain}">
                ${options}
              </select>
            </div>
          `;
        }

        if (domain === "number") {
          const min = state.attributes?.min || 0;
          const max = state.attributes?.max || 100;
          const step = state.attributes?.step || 1;
          return `
            <div class="entity-row">
              <div class="entity-name">${name}</div>
              <input type="number" class="entity-control" data-entity="${entityId}" data-domain="${domain}"
                value="${stateValue}" min="${min}" max="${max}" step="${step}" />
            </div>
          `;
        }

        return `
          <div class="entity-row">
            <div class="entity-name">${name}</div>
            <div class="entity-state">${stateValue}${unit ? " " + unit : ""}</div>
          </div>
        `;
      })
      .join("");

    setTimeout(() => {
      modalBody.querySelectorAll(".entity-control").forEach(control => {
        control.addEventListener("change", async e => {
          const entityId = e.target.dataset.entity;
          const domain = e.target.dataset.domain;
          const value = e.target.value;

          if (domain === "select") {
            await hass.callService("select", "select_option", {
              entity_id: entityId,
              option: value
            });
          } else if (domain === "number") {
            await hass.callService("number", "set_value", {
              entity_id: entityId,
              value: parseFloat(value)
            });
          }

          setTimeout(() => hass.refresh(), 500);
        });
      });
    }, 0);

    modal.classList.add("active");
  };
}
