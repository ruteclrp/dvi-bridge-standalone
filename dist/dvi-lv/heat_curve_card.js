export const HEAT_CURVE_DEFAULTS = {
	cv_curve: null,
	curve_set_minus12: null,
	curve_set_plus12: null,
	outdoor: null,
	curve_sensor: null,
	cv_min: null,
	cv_max: null,
};

const yTicksPlugin = {
	id: "forceYTicks",
	afterBuildTicks: (chart) => {
		const yScale = chart.scales.y;
		if (!yScale) return;
		const ticks = [];
		for (let v = 20; v <= 60; v += 10) ticks.push({ value: v });
		yScale.ticks = ticks;
		yScale._tickItems = ticks.map((t) => ({ value: t.value }));
	},
};

class HeatCurveCard extends HTMLElement {
	constructor() {
		super();
		this._entities = { ...HEAT_CURVE_DEFAULTS };
		this._title = "Kurvetemperatur";
		this._pendingHass = null;
		this._pendingConfig = null;
		this._controlsCard = null;
	}

	async connectedCallback() {
		await this._ensureChartLib();
		this._renderSkeleton();
		this._ensureChart();
		// if setConfig was called before connect, apply it now
		if (this._pendingConfig) {
			const cfg = this._pendingConfig;
			this._pendingConfig = null;
			this.setConfig(cfg);
		}
		if (this._pendingHass) {
			const pending = this._pendingHass;
			this._pendingHass = null;
			this.hass = pending;
		}
	}

	setConfig(config) {
		this.config = config || {};
		this._title = this.config.title || "Kurvetemperatur";
		// store config while not connected so popup/ordering doesn't matter
		if (!this.isConnected) {
			this._pendingConfig = config;
			return;
		}

		// apply config now we're connected
		if (config?.entities) {
			// support both object-form ({ cv_curve: "sensor.x" }) and array-form (["sensor.cv", ...])
			if (Array.isArray(config.entities)) {
				const keys = Object.keys(HEAT_CURVE_DEFAULTS);
				this._entities = { ...HEAT_CURVE_DEFAULTS };
				config.entities.forEach((eid, i) => {
					if (eid) this._entities[keys[i]] = eid;
				});
			} else {
				this._entities = {
					...HEAT_CURVE_DEFAULTS,
					...Object.fromEntries(
						Object.entries(config.entities).filter(([, value]) => !!value)
					),
				};
			}
		}
		this._renderSkeleton();
		this._ensureChart(true);
	}

	set hass(hass) {
		if (!this.isConnected) {
			this._pendingHass = hass;
			return;
		}
		this._hass = hass;

		// try auto-mapping missing entities once when hass arrives
		if (this._hass && (!this._entities.cv_curve || !this._entities.curve_set_minus12 || !this._entities.curve_set_plus12)) {
			this._autoMapEntities();
		}

		const values = this._readValues();
		// update chart only if we have valid values; always render controls (so popup shows entities)
		if (values) {
			this._updateChart(values);
		} else if (this.chart) {
			// clear chart if present and no valid values
			this.chart.data.labels = [];
			this.chart.data.datasets.forEach((d) => (d.data = []));
			this.chart.update();
		}
		this._renderControls();
	}

	async _ensureChartLib() {
		if (!window.Chart) {
			await import("https://cdn.jsdelivr.net/npm/chart.js@4.4.1/dist/chart.umd.min.js");
		}
	}

	_renderSkeleton() {		
		const existingCard = this.querySelector("ha-card");
		if (!existingCard) {
			this.innerHTML = `
        
        <ha-card class="heat-curve-card">
          <div class="heat-curve-card__chart" data-role="chart">
            <canvas data-role="curveChart"></canvas>
          </div>
          <div class="heat-curve-card__controls" data-role="controls"></div>
        </ha-card>
      `;
		} else {
			existingCard.removeAttribute("header");
		}
		this._canvas = this.querySelector('[data-role="curveChart"]');
		this._controls = this.querySelector('[data-role="controls"]');
	}

	_ensureChart(reset = false) {
		if (!this._canvas || !window.Chart) return;

		if (reset && this.chart) {
			this.chart.destroy();
			this.chart = null;
		}
		if (this.chart) return;

		const ctx = this._canvas.getContext("2d");
		this.chart = new Chart(ctx, {
			type: "line",
			data: {
				labels: [],
				datasets: [
					{
						label: "CV Temp Curve",
						data: [],
						borderColor: "#e04e4e",
						backgroundColor: "rgba(224,78,78,0.08)",
						fill: true,
						borderWidth: 2,
						tension: 0.28,        // slight smoothing
						pointRadius: 0,      // no points on the curve line
						pointHoverRadius: 4,
						pointBackgroundColor: "#e04e4e",
                     },
                     {
                         label: "Sensor Curve Temp",
                         data: [],
                         borderColor: "#2b7bd3",
                         pointBackgroundColor: "#2b7bd3",
                         showLine: false,
                         pointRadius: 5,
                         pointHoverRadius: 7,
                     },
                 ],
             },
             options: {
				responsive: true,
				maintainAspectRatio: false,
				animation: false,
				interaction: { mode: "index", intersect: false },
				scales: {
					x: {
						title: { display: true, text: "Outdoor Temp (°C)" },
						min: -20,
						max: 32,
						grid: { color: "rgba(0,0,0,0.04)" },
						ticks: { color: "#444", maxRotation: 0 },
					},
					y: {
						title: { display: true, text: "CV Temp (°C)" },
						min: 10,
						max: 80,
						grid: { drawTicks: true, color: "rgba(0,0,0,0.04)" },
						ticks: { callback: (v) => `${v}°C`, color: "#444" },
					},
				},
				plugins: {
					legend: {
						display: true,
						position: "top",
						align: "start",
						labels: {
							usePointStyle: true,
							pointStyle: "circle",
							boxWidth: 10,
							padding: 10,
							color: "#333",
							font: { size: 12 },
						},
					},
					tooltip: {
						enabled: true,
						mode: "index",
						intersect: false,
						backgroundColor: "rgba(0,0,0,0.78)",
						titleColor: "#fff",
						bodyColor: "#fff",
						callbacks: {
							label: (ctx) => {
								const y = ctx.parsed?.y;
								return `${ctx.dataset.label}: ${y !== undefined ? `${y}°C` : ""}`;
							},
						},
					},
				},
				elements: {
					line: { tension: 0.28, borderJoinStyle: "round" },
					point: { hoverBorderWidth: 2 },
				},
				layout: { padding: { top: 6, right: 8, bottom: 4, left: 4 } },
			},
             plugins: [yTicksPlugin],
         });
	}

	_readValues() {
		if (!this._hass) return null;

		const readNumber = (key) => {
            const entityId = this._entities[key];
            if (!entityId) return NaN;
            const stateObj = this._hass.states?.[entityId];
            if (!stateObj) {
                return NaN;
            }
            // try top-level state first, then common attributes
            let raw = stateObj.state;
            const a = stateObj.attributes || {};
            if (raw === undefined || raw === null || isNaN(parseFloat(String(raw).replace(",", ".")))) {
                raw = a.native_value ?? a.value ?? a.state ?? raw;
            }
            // normalize comma decimal and extract first numeric token (handles "23.4 °C")
            const cleaned = String(raw).replace(",", ".");
            const m = cleaned.match(/-?\d+(\.\d+)?/);
            const num = m ? parseFloat(m[0]) : NaN;
            return num;
        };

         const cvCurve = readNumber("cv_curve");
         const setPlus12 = readNumber("curve_set_plus12");
         const setMinus12 = readNumber("curve_set_minus12");
         const outdoorTemp = readNumber("outdoor");
         const curveTemp = readNumber("curve_sensor");
         const cvMax = readNumber("cv_max");
         const cvMin = readNumber("cv_min");
 
		// debug summary if required inputs are not valid
		const required = { cvCurve, setPlus12, setMinus12, outdoorTemp, curveTemp };
		const missingKeys = Object.entries(required).filter(([, v]) => Number.isNaN(v)).map(([k]) => k);
		if (missingKeys.length) {
			return null;
		}
 
         return {
             cvCurve,
             setPlus12,
             setMinus12,
             outdoorTemp,
             curveTemp,
             cvMax: Number.isNaN(cvMax) ? undefined : cvMax,
             cvMin: Number.isNaN(cvMin) ? undefined : cvMin,
         };
     }

	_updateChart(values) {
		if (!this.chart) return;

		const temps = Array.from({ length: 53 }, (_, i) => i - 20);
		const shift = values.cvCurve - 10;
		const adjMinus12 = values.setMinus12 + shift;
		const adjPlus12 = values.setPlus12 + shift;
		const slope = (adjPlus12 - adjMinus12) / 24;
		const cvMin = Number.isFinite(values.cvMin) ? values.cvMin : 20;
		const cvMax = Number.isFinite(values.cvMax) ? values.cvMax : 55;

		const cvTemps = temps.map((t) => {
			const val = adjMinus12 + slope * (t + 12);
			return Math.max(cvMin, Math.min(cvMax, val));
		});

		this.chart.data.labels = temps;
		this.chart.data.datasets[0].data = cvTemps;

		const sensorData = new Array(temps.length).fill(null);
		const idx = temps.indexOf(Math.round(values.outdoorTemp));
		if (idx !== -1) sensorData[idx] = values.curveTemp;
		this.chart.data.datasets[1].data = sensorData;

		this.chart.update();
	}

	_renderControls() {
		if (!this._controls || !this._hass) return;
 
        const entities = [
            this._entities.cv_curve,
            this._entities.curve_set_minus12,
            this._entities.curve_set_plus12,
            this._entities.cv_min,
            this._entities.cv_max,
         ].filter(Boolean);

		if (!entities.length) {
			this._controls.innerHTML = "";
			this._controlsCard = null;
			return;
		}

		if (!this._controlsCard) {
			this._controls.innerHTML = "";
			this._controlsCard = document.createElement("hui-entities-card");
			this._controls.appendChild(this._controlsCard);
		}

		this._controlsCard.setConfig({ entities });
		this._controlsCard.hass = this._hass;
	}

	_autoMapEntities() {
        if (!this._hass) return;
        const all = Object.keys(this._hass.states || {});
        // focus on DVI entities to avoid false matches
        const dvi = all.filter((id) => id.includes("dvi_lv12"));

        const pick = (pred) => dvi.find(pred);
        const picks = (pred) => dvi.filter(pred);

        // cv_curve
        if (!this._entities.cv_curve) {
            this._entities.cv_curve = pick((id) => id.includes("cv_curve") || id.includes("cvcurve") || id.includes("cv_curve"));
        }
        // curve set - try to find two variants (one with _2 for plus)
        if (!this._entities.curve_set_minus12 || !this._entities.curve_set_plus12) {
            const sets = picks((id) => id.includes("curve_set") || id.includes("curve_set_12"));
            if (sets.length === 1) {
                this._entities.curve_set_minus12 = this._entities.curve_set_minus12 || sets[0];
            } else if (sets.length >= 2) {
                // try to pick the "_2" as plus12
                const plus = sets.find((s) => s.includes("_2")) || sets[1];
                const minus = sets.find((s) => !s.includes("_2")) || sets[0];
                this._entities.curve_set_minus12 = this._entities.curve_set_minus12 || minus;
                this._entities.curve_set_plus12 = this._entities.curve_set_plus12 || plus;
            }
        }
        // cv_min / cv_max
        if (!this._entities.cv_min) this._entities.cv_min = pick((id) => id.includes("cv_min"));
        if (!this._entities.cv_max) this._entities.cv_max = pick((id) => id.includes("cv_max"));
        // outdoor / curve_sensor fallback (only if missing)
        if (!this._entities.outdoor) this._entities.outdoor = pick((id) => id.includes("outdoor"));
        if (!this._entities.curve_sensor) this._entities.curve_sensor = pick((id) => id.includes("curve_temp") || id.includes("curve"));
    }
}

customElements.define("heat-curve-card", HeatCurveCard);
