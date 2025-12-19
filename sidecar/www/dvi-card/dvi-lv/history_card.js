class HistoryCard extends HTMLElement {
  constructor() {
    super();
    this._timeSpan = 24; // Default to 24 hours
    this._fullData = null; // Store full dataset
    this._chart = null; // Store chart instance
  }
  
  setConfig(config) {
    this._config = config;
    this.render();
  }

  set hass(hass) {
    this._hass = hass;
    this.loadHistoryData();
  }

  async loadHistoryData() {
    if (!this._config || !this._config.sensor) {
      console.log('❌ History card: No config or sensor');
      return;
    }
    
    console.log(`📊 Loading history for sensor: ${this._config.sensor}`);
    
    try {
      const url = `/api/history/${encodeURIComponent(this._config.sensor)}`;
      console.log(`🌐 Fetching: ${url}`);
      
      const response = await fetch(url);
      console.log(`📥 Response status: ${response.status}`);
      
      const data = await response.json();
      console.log(`📦 Response data:`, data);
      
      if (data.data && data.data.length > 0) {
        console.log(`✅ Got ${data.data.length} data points`);
        this._fullData = data.data; // Store full dataset
        this.updateChart();
      } else {
        console.log(`⚠️ No data available`);
        this.showNoData();
      }
    } catch (err) {
      console.error("❌ Failed to load history:", err);
      this.showError(err.message);
    }
  }
  
  updateChart() {
    if (!this._fullData) return;
    
    // Filter data based on selected time span
    const now = Date.now() / 1000;
    const cutoff = now - (this._timeSpan * 3600); // Convert hours to seconds
    const filteredData = this._fullData.filter(point => point.timestamp >= cutoff);
    
    if (filteredData.length > 0) {
      this.renderChart(filteredData);
    } else {
      this.showNoData();
    }
  }
  
  setTimeSpan(hours) {
    this._timeSpan = hours;
    // Update active button styling
    this.querySelectorAll('.time-span-btn').forEach(btn => {
      const isActive = parseInt(btn.dataset.hours) === hours;
      btn.classList.toggle('active', isActive);
      
      // Update inline styles
      if (isActive) {
        btn.style.background = '#2b7bd3';
        btn.style.borderColor = '#2b7bd3';
        btn.style.color = 'white';
      } else {
        btn.style.background = 'white';
        btn.style.borderColor = '#ddd';
        btn.style.color = '#333';
      }
    });
    this.updateChart();
  }

  render() {
    this.innerHTML = `
      <ha-card>
        <div class="history-card">
          <div class="history-card__controls" style="padding: 8px 16px; display: flex; gap: 8px; justify-content: center; border-bottom: 1px solid rgba(0,0,0,0.1);">
            <button class="time-span-btn" data-hours="1" style="padding: 4px 12px; border: 1px solid #ddd; background: white; border-radius: 4px; cursor: pointer; font-size: 12px;">1h</button>
            <button class="time-span-btn" data-hours="3" style="padding: 4px 12px; border: 1px solid #ddd; background: white; border-radius: 4px; cursor: pointer; font-size: 12px;">3h</button>
            <button class="time-span-btn" data-hours="6" style="padding: 4px 12px; border: 1px solid #ddd; background: white; border-radius: 4px; cursor: pointer; font-size: 12px;">6h</button>
            <button class="time-span-btn" data-hours="12" style="padding: 4px 12px; border: 1px solid #ddd; background: white; border-radius: 4px; cursor: pointer; font-size: 12px;">12h</button>
            <button class="time-span-btn active" data-hours="24" style="padding: 4px 12px; border: 1px solid #2b7bd3; background: #2b7bd3; color: white; border-radius: 4px; cursor: pointer; font-size: 12px;">24h</button>
          </div>
          <div class="history-card__chart" style="height: 300px; padding: 16px;">
            <canvas id="history-chart-${this._config.sensor}"></canvas>
          </div>
        </div>
      </ha-card>
    `;
    
    // Add click handlers to buttons
    this.querySelectorAll('.time-span-btn').forEach(btn => {
      btn.onclick = () => this.setTimeSpan(parseInt(btn.dataset.hours));
      // Add hover effect
      btn.onmouseenter = (e) => {
        if (!e.target.classList.contains('active')) {
          e.target.style.background = '#f5f5f5';
        }
      };
      btn.onmouseleave = (e) => {
        if (!e.target.classList.contains('active')) {
          e.target.style.background = 'white';
        }
      };
    });
    
    // Style active button
    const activeBtn = this.querySelector('.time-span-btn.active');
    if (activeBtn) {
      activeBtn.style.background = '#2b7bd3';
      activeBtn.style.borderColor = '#2b7bd3';
      activeBtn.style.color = 'white';
    }
  }

  showNoData() {
    const canvas = this.querySelector("canvas");
    if (canvas) {
      canvas.parentElement.innerHTML = '<div style="padding: 40px; text-align: center; color: #999;">No history data available yet. Data is collected every 60 seconds.</div>';
    }
  }

  showError(message) {
    const canvas = this.querySelector("canvas");
    if (canvas) {
      canvas.parentElement.innerHTML = `<div style="padding: 40px; text-align: center; color: #f44336;">Error: ${message}</div>`;
    }
  }

  renderChart(data) {
    const canvas = this.querySelector(`#history-chart-${this._config.sensor}`);
    if (!canvas) return;

    // Destroy existing chart if it exists
    if (this._chart) {
      this._chart.destroy();
      this._chart = null;
    }

    const ctx = canvas.getContext("2d");
    
    // Sort by timestamp
    data.sort((a, b) => a.timestamp - b.timestamp);
    
    // Prepare chart data
    const labels = data.map((point, idx) => {
      const date = new Date(point.timestamp * 1000);
      return date.toLocaleTimeString('en-GB', { hour: '2-digit', minute: '2-digit', hour12: false });
    });
    
    const values = data.map(point => point.value);
    
    this._chart = new Chart(ctx, {
      type: 'line',
      data: {
        labels: labels,
        datasets: [{
          label: this._config.title || this._config.sensor,
          data: values,
          borderColor: '#2b7bd3',
          backgroundColor: 'rgba(43, 123, 211, 0.08)',
          fill: true,
          borderWidth: 2,
          tension: 0.28,
          pointRadius: 0,
          pointHoverRadius: 4,
          pointBackgroundColor: '#2b7bd3'
        }]
      },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        animation: false,
        interaction: { mode: 'index', intersect: false },
        plugins: {
          legend: {
            display: true,
            position: 'top',
            align: 'end',
            labels: {
              usePointStyle: true,
              pointStyle: 'circle',
              boxWidth: 10,
              padding: 10,
              color: '#333',
              font: { size: 12 }
            }
          },
          tooltip: {
            enabled: true,
            mode: 'index',
            intersect: false,
            backgroundColor: 'rgba(0,0,0,0.78)',
            titleColor: '#fff',
            bodyColor: '#fff',
            callbacks: {
              label: (ctx) => {
                const y = ctx.parsed?.y;
                return `${ctx.dataset.label}: ${y !== undefined ? `${y.toFixed(1)}°C` : ''}`;
              }
            }
          }
        },
        scales: {
          x: {
            title: { 
              display: true, 
              text: `Time (${this._timeSpan}h)`,
              color: '#444'
            },
            grid: { color: 'rgba(0,0,0,0.04)' },
            ticks: { 
              color: '#444',
              maxRotation: 0,
              maxTicksLimit: 12,
              autoSkip: true
            }
          },
          y: {
            title: { 
              display: true, 
              text: 'Temperature (°C)',
              color: '#444'
            },
            grid: { 
              drawTicks: true,
              color: 'rgba(0,0,0,0.04)' 
            },
            ticks: { 
              callback: (v) => `${v}°C`,
              color: '#444'
            }
          }
        },
        elements: {
          line: { 
            tension: 0.28, 
            borderJoinStyle: 'round' 
          },
          point: { hoverBorderWidth: 2 }
        },
        layout: { 
          padding: { top: 6, right: 8, bottom: 4, left: 4 } 
        }
      }
    });
  }
}

if (!customElements.get("history-card")) {
  customElements.define("history-card", HistoryCard);
}

export { HistoryCard };
