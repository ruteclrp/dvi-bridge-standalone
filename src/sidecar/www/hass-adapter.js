export class HassAdapter {
  constructor() {
    this.states = {};
    this.popupCallback = null; // Will be set by index.html
  }

  async refresh() {
    const res = await fetch("/api/states");
    this.states = await res.json();
  }

  callService(domain, service, data) {
    // Intercept browser_mod popup calls
    if (domain === "browser_mod" && service === "popup") {
      if (this.popupCallback) {
        this.popupCallback(data);
      }
      return Promise.resolve();
    }

    // Intercept service calls for entity controls
    if (this.serviceCallback) {
      this.serviceCallback(domain, service, data);
    }

    return fetch(`/api/services/${domain}/${service}`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(data),
    });
  }
}
