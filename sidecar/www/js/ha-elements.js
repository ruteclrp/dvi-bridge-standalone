const supportsAdoptedStyleSheets = "adoptedStyleSheets" in Document.prototype;
const haIconBaseSheet = supportsAdoptedStyleSheets ? new CSSStyleSheet() : null;

if (haIconBaseSheet) {
  haIconBaseSheet.replaceSync(`
    :host {
      display: inline-block;
      width: 24px;
      height: 24px;
    }
    i {
      font-size: 24px;
      line-height: 1;
      color: inherit;
    }
  `);
}

let mdiSheetPromise;
function loadMdiSheetOnce() {
  if (!supportsAdoptedStyleSheets) return Promise.resolve(null);
  if (!mdiSheetPromise) {
    mdiSheetPromise = (async () => {
      if (window.__mdiSharedSheet) return window.__mdiSharedSheet;
      const response = await fetch("/css/materialdesignicons.min.css");
      const cssText = (await response.text()).replace(/\/\*# sourceMappingURL=[^*]+?\*\//g, "");
      const sheet = new CSSStyleSheet();
      sheet.replaceSync(cssText);
      window.__mdiSharedSheet = sheet;
      return sheet;
    })();
  }
  return mdiSheetPromise;
}

class HaIcon extends HTMLElement {
  static get observedAttributes() {
    return ["icon"];
  }

  constructor() {
    super();
    this._usesShadow = supportsAdoptedStyleSheets;
    this._root = this._usesShadow ? this.attachShadow({ mode: "open" }) : this;
    this._iconEl = document.createElement("i");
    this._root.appendChild(this._iconEl);

    if (this._usesShadow && haIconBaseSheet) {
      this._root.adoptedStyleSheets = [haIconBaseSheet];
      loadMdiSheetOnce()
        .then(sheet => {
          if (sheet && !this._root.adoptedStyleSheets.includes(sheet)) {
            this._root.adoptedStyleSheets = [haIconBaseSheet, sheet];
          }
        })
        .catch(err => console.error("HaIcon CSS load failed", err));
    } else if (!this._usesShadow) {
      Object.assign(this.style, {
        display: "inline-block",
        width: "24px",
        height: "24px"
      });
      Object.assign(this._iconEl.style, {
        fontSize: "24px",
        lineHeight: "1",
        color: "inherit"
      });
    }
  }

  connectedCallback() {
    this._render();
  }

  attributeChangedCallback() {
    this._render();
  }

  _render() {
    const icon = (this.getAttribute("icon") || "").replace("mdi:", "mdi-");
    this._iconEl.className = icon ? `mdi ${icon}` : "mdi";
  }
}
customElements.define("ha-icon", HaIcon);

class HaCard extends HTMLElement {
  connectedCallback() {
    const header = this.getAttribute("header");
    if (!this.querySelector(".ha-card-header") && header) {
      const headerEl = document.createElement("div");
      headerEl.className = "ha-card-header";
      headerEl.textContent = header;
      this.prepend(headerEl);
    }
  }

  static get observedAttributes() {
    return ["header"];
  }

  attributeChangedCallback(name, _oldValue, newValue) {
    if (name === "header" && this.isConnected) {
      let headerEl = this.querySelector(".ha-card-header");
      if (!headerEl) {
        headerEl = document.createElement("div");
        headerEl.className = "ha-card-header";
        this.prepend(headerEl);
      }
      headerEl.textContent = newValue;
    }
  }
}
customElements.define("ha-card", HaCard);

class HuiEntitiesCard extends HTMLElement {
  constructor() {
    super();
    this._config = null;
    this._hass = null;
  }

  setConfig(config) {
    this._config = config;
    this._render();
  }

  set hass(hass) {
    this._hass = hass;
    this._render();
  }

  _render() {
    if (!this._config || !this._hass) return;

    const entities = this._config.entities || [];
    this.innerHTML = entities
      .map(entityId => {
        const state = this._hass.states?.[entityId];
        if (!state) return "";

        const name = state.attributes?.friendly_name || entityId;
        const stateValue = state.state;
        const domain = entityId.split(".")[0];

        if (domain === "number") {
          const min = state.attributes?.min || 0;
          const max = state.attributes?.max || 100;
          const step = state.attributes?.step || 0.1;
          const unit = state.attributes?.unit_of_measurement || "";
          return `
            <div class="entity-row">
              <div class="entity-name">${name}</div>
              <div style="display: flex; align-items: center; gap: 8px;">
                <input type="number" class="entity-control heat-curve-control"
                  data-entity="${entityId}"
                  value="${stateValue}"
                  min="${min}"
                  max="${max}"
                  step="${step}"
                  style="width: 100px;" />
                ${unit ? `<span style="color: #999;">${unit}</span>` : ""}
              </div>
            </div>
          `;
        }

        return `
          <div class="entity-row">
            <div class="entity-name">${name}</div>
            <div class="entity-state">${stateValue}</div>
          </div>
        `;
      })
      .join("");

    this.querySelectorAll(".heat-curve-control").forEach(control => {
      control.addEventListener("change", async e => {
        const entityId = e.target.dataset.entity;
        const value = parseFloat(e.target.value);

        await this._hass.callService("number", "set_value", {
          entity_id: entityId,
          value
        });

        setTimeout(() => this._hass.refresh?.(), 500);
      });
    });
  }
}
customElements.define("hui-entities-card", HuiEntitiesCard);
