export class HassAdapter {
  constructor() {
    this.states = {};
    this.popupCallback = null; // Will be set by index.html
  }

  async refresh() {
    try {
      const res = await fetch("/api/states");
      
      // Handle authentication errors
      if (res.status === 401) {
        this.handleAuthRequired();
        throw new Error("Authentication required");
      }
      
      this.states = await res.json();
    } catch (error) {
      if (error.message !== "Authentication required") {
        console.error("Failed to refresh states:", error);
      }
      throw error;
    }
  }

  async callService(domain, service, data) {
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

    const res = await fetch(`/api/services/${domain}/${service}`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(data),
    });
    
    // Handle authentication errors
    if (res.status === 401) {
      this.handleAuthRequired();
      throw new Error("Authentication required");
    }
    
    return res;
  }

  handleAuthRequired() {
    // Trigger login UI (will be set by app.js)
    if (window.showLoginModal) {
      window.showLoginModal();
    }
  }
}
