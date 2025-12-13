export class HassAdapter {
  constructor() {
    this.states = {};
  }

  async refresh() {
    const res = await fetch("/api/states");
    this.states = await res.json();
  }

  callService(domain, service, data) {
    return fetch(`/api/services/${domain}/${service}`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(data),
    });
  }
}
