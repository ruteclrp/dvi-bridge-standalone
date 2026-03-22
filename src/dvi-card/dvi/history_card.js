class HistoryCard extends HTMLElement {
  constructor() {
    super();
    this._selectedPreset = "today";
    this._selectedDate = this.todayLocalIso();
    this._fromHour = 0;
    this._toHour = 24;
    this._fullData = null;
    this._defrostData = null;
    this._period = null;
    this._chart = null;
  }
  
  connectedCallback() {
    // Reset modal sizing when history card is opened
    // This prevents inheriting fixed widths from heat curve popup
    const modalBody = this.closest('.modal-body');
    if (modalBody) {
      const modalContent = modalBody.closest('.modal-content');
      if (modalContent) {
        // Reset to CSS defaults for history cards, but constrain width
        modalContent.style.width = 'fit-content';
        modalContent.style.minWidth = '400px';
        modalContent.style.maxWidth = '550px';
      }
      // Reset modal-body padding
      modalBody.style.paddingLeft = '';
      modalBody.style.paddingRight = '';
    }
    
    // Set width constraints on ha-card to prevent Chart.js from expanding it
    setTimeout(() => {
      const haCard = this.querySelector('ha-card');
      if (haCard) {
        haCard.style.width = '500px';
        haCard.style.maxWidth = '500px';
        haCard.style.minWidth = '400px';
      }
    }, 0);
  }
  
  formatDuration(seconds) {
    const hours = Math.floor(seconds / 3600);
    const minutes = Math.floor((seconds % 3600) / 60);
    const secs = Math.floor(seconds % 60);
    
    if (hours > 0) {
      return `${hours}h ${minutes}m`;
    } else if (minutes > 0) {
      return `${minutes}m ${secs}s`;
    } else {
      return `${secs}s`;
    }
  }

  toIsoLocalDate(date) {
    const year = date.getFullYear();
    const month = String(date.getMonth() + 1).padStart(2, '0');
    const day = String(date.getDate()).padStart(2, '0');
    return `${year}-${month}-${day}`;
  }

  todayLocalIso() {
    return this.toIsoLocalDate(new Date());
  }

  minSelectableDate() {
    const date = new Date();
    date.setDate(date.getDate() - 29);
    return this.toIsoLocalDate(date);
  }

  formatDateDa(dateString) {
    if (!dateString) return '?';
    const [year, month, day] = String(dateString).split('-');
    if (!year || !month || !day) return dateString;
    return `${day}/${month}/${year}`;
  }

  rangeLabel(range) {
    const labels = {
      today: 'I dag',
      yesterday: 'I går',
      custom: 'Valgt dag',
    };
    return labels[range] || range;
  }

  getPresetDate(preset) {
    const today = new Date();
    if (preset === 'yesterday') {
      today.setDate(today.getDate() - 1);
    }
    return this.toIsoLocalDate(today);
  }

  getSelectedWindow() {
    if (!this._selectedDate || !this._period) return null;

    const [year, month, day] = this._selectedDate.split('-').map(Number);
    const start = new Date(year, month - 1, day, this._fromHour, 0, 0, 0);
    const end = new Date(year, month - 1, day, this._toHour, 0, 0, 0);
    const periodStart = this._period.start_timestamp;
    const periodEnd = this._period.end_timestamp;

    return {
      startTimestamp: Math.max(start.getTime() / 1000, periodStart),
      endTimestamp: Math.min(end.getTime() / 1000, periodEnd),
    };
  }

  formatHourLabel(hour) {
    return `${String(hour).padStart(2, '0')}:00`;
  }

  applyTimeRange() {
    if (this._fromHour >= this._toHour) {
      this.showError('Fra tid skal være tidligere end til tid');
      return;
    }

    if (this._config.sensor === 'defrost' || this._config.sensor === 'aux_heating') {
      this.updateDefrostOnly();
    } else {
      this.updateChart();
    }
  }

  setFromHour(hour) {
    this._fromHour = hour;
    this.render();
    this.applyTimeRange();
  }

  setToHour(hour) {
    this._toHour = hour;
    this.render();
    this.applyTimeRange();
  }
  
  setConfig(config) {
    this._config = config;
    this._selectedPreset = config?.default_preset || 'today';
    this._selectedDate = config?.selected_date || this.getPresetDate(this._selectedPreset);
    this.render();
  }

  set hass(hass) {
    this._hass = hass;
    this.loadHistoryData();
  }

  async loadHistoryData() {
    if (!this._config || !this._config.sensor) {
      return;
    }

    if (!this._selectedDate) {
      this.showError('Vælg en gyldig dato');
      return;
    }

    this._period = null;
    this._fullData = null;
    this._defrostData = null;
    
    try {
      const url = `/api/history/${encodeURIComponent(this._config.sensor)}?date=${encodeURIComponent(this._selectedDate)}`;
      const response = await fetch(url);
      const data = await response.json();

      if (!response.ok) {
        throw new Error(data?.error || 'Failed to load history');
      }

      this._period = data.period || null;

      if (this._config.sensor === 'defrost' || this._config.sensor === 'aux_heating') {
        this._defrostData = data.data || [];
        this.updateDefrostOnly();
        return;
      }

      this._fullData = data.data || [];

      if (this._config.sensor === 'evaporator_temp') {
        const defrostUrl = `/api/history/defrost?date=${encodeURIComponent(this._selectedDate)}`;
        const defrostResponse = await fetch(defrostUrl);
        if (defrostResponse.ok) {
          const defrostData = await defrostResponse.json();
          this._defrostData = defrostData.data || [];
        }
      }

      this.updateChart();
    } catch (err) {
      this.showError(err.message);
    }
  }
  
  updateDefrostOnly() {
    if (!this._defrostData) return;

    const selectedWindow = this.getSelectedWindow();
    const startTime = selectedWindow?.startTimestamp;
    const endTime = selectedWindow?.endTimestamp;
    if (!startTime || !endTime) {
      this.showNoData();
      return;
    }

    if (startTime >= endTime) {
      this.showError('Fra tid skal være tidligere end til tid');
      return;
    }

    this.renderDefrostTimeline(startTime, endTime);
  }
  
  updateChart() {
    if (!this._fullData || this._fullData.length === 0) {
      this.showNoData();
      return;
    }

    const selectedWindow = this.getSelectedWindow();
    const startTime = selectedWindow?.startTimestamp;
    const endTime = selectedWindow?.endTimestamp;
    if (!startTime || !endTime || startTime >= endTime) {
      this.showError('Fra tid skal være tidligere end til tid');
      return;
    }

    const filteredData = this._fullData.filter(point =>
      point.timestamp >= startTime && point.timestamp <= endTime
    );

    if (filteredData.length === 0) {
      this.showNoData();
      return;
    }

    this.renderChart(filteredData);
    if (this._config.sensor === 'evaporator_temp' && this._defrostData && this._period) {
      this.renderDefrostTimeline(startTime, endTime);
    }
  }
  
  renderDefrostTimeline(startTime, endTime) {
    if (!this._defrostData || this._defrostData.length === 0) return;
    
    const barsContainer = this.querySelector(`#defrost-bars-${this._config.sensor}`);
    if (!barsContainer) return;
    
    // Clear existing bars
    barsContainer.innerHTML = '';
    
    const duration = endTime - startTime;
    
    // Filter defrost data to selected date window
    const filteredDefrost = this._defrostData.filter(point => 
      point.timestamp >= startTime && point.timestamp <= endTime
    );
    
    // Convert to periods (start/end of active defrost)
    const periods = [];
    let periodStart = null;
    
    for (let i = 0; i < filteredDefrost.length; i++) {
      const current = filteredDefrost[i];
      const isActive = current.value === 1;
      
      if (isActive && periodStart === null) {
        periodStart = current.timestamp;
      } else if (!isActive && periodStart !== null) {
        periods.push({ start: periodStart, end: current.timestamp });
        periodStart = null;
      }
    }
    
    // Close any open period
    if (periodStart !== null) {
      periods.push({ start: periodStart, end: endTime });
    }
    
    // Get tooltip element
    const tooltip = this.querySelector(`#bar-tooltip-${this._config.sensor}`);
    
    // Render bars for each period
    periods.forEach(period => {
      const startPercent = ((period.start - startTime) / duration) * 100;
      const widthPercent = ((period.end - period.start) / duration) * 100;
      
      const bar = document.createElement('div');
      bar.style.cssText = `
        position: absolute;
        left: ${startPercent}%;
        width: ${widthPercent}%;
        height: 100%;
        background: #03a9f4;
        opacity: 0.8;
        cursor: pointer;
      `;
      
      // Add hover handlers for tooltip
      if (tooltip) {
        const durationSeconds = period.end - period.start;
        const startDate = new Date(period.start * 1000);
        const endDate = new Date(period.end * 1000);
        const startTime = startDate.toLocaleTimeString('en-GB', { hour: '2-digit', minute: '2-digit', hour12: false });
        const endTime = endDate.toLocaleTimeString('en-GB', { hour: '2-digit', minute: '2-digit', hour12: false });
        const durationStr = this.formatDuration(durationSeconds);
        
        bar.addEventListener('mouseenter', (e) => {
          tooltip.textContent = `${startTime} → ${endTime}  (${durationStr})`;
          tooltip.style.display = 'block';
          
          // Position tooltip above the bar
          const rect = bar.getBoundingClientRect();
          const tooltipRect = tooltip.getBoundingClientRect();
          const containerRect = barsContainer.getBoundingClientRect();
          
          tooltip.style.left = `${rect.left - containerRect.left + (rect.width / 2) - (tooltipRect.width / 2)}px`;
          tooltip.style.top = `${rect.top - containerRect.top - tooltipRect.height - 8}px`;
        });
        
        bar.addEventListener('mouseleave', () => {
          tooltip.style.display = 'none';
        });
      }
      
      barsContainer.appendChild(bar);
    });
    
    // Add time labels below the bar
    const timelineContainer = this.querySelector(`#defrost-timeline-${this._config.sensor}`);
    if (!timelineContainer) return;
    
    // Remove existing labels
    const existingLabels = timelineContainer.querySelector('.defrost-time-labels');
    if (existingLabels) existingLabels.remove();
    
    // Create time labels container
    const labelsDiv = document.createElement('div');
    labelsDiv.className = 'defrost-time-labels';
    const isDefrostOnly = this._config.sensor === 'defrost';
    labelsDiv.style.cssText = `
      display: flex;
      justify-content: space-between;
      padding-top: 4px;
      font-size: 10px;
      color: #999;
      padding-left: ${isDefrostOnly ? '0' : '60px'};
      padding-right: 12px;
    `;
    
    const numLabels = 8;
    
    for (let i = 0; i < numLabels; i++) {
      const timestamp = startTime + (duration * i / (numLabels - 1));
      const date = new Date(timestamp * 1000);
      const timeStr = date.toLocaleTimeString('en-GB', { hour: '2-digit', minute: '2-digit', hour12: false });
      
      const label = document.createElement('span');
      label.textContent = timeStr;
      labelsDiv.appendChild(label);
    }
    
    timelineContainer.appendChild(labelsDiv);
  }
  
  setPreset(preset) {
    this._selectedPreset = preset;
    this._selectedDate = this.getPresetDate(preset);
    this.render();
    this.loadHistoryData();
  }

  setSelectedDate(value) {
    if (!value) return;
    this._selectedPreset = 'custom';
    this._selectedDate = value;
    this.render();
    this.loadHistoryData();
  }

  render() {
    const isDefrostOnly = this._config.sensor === 'defrost' || this._config.sensor === 'aux_heating';
    const showDefrostTimeline = isDefrostOnly || this._config.sensor === 'evaporator_temp';
    
    this.innerHTML = `
      <ha-card>
        <div class="history-card">
          <div class="history-card__controls" style="padding: 10px 16px; display: grid; gap: 10px; border-bottom: 1px solid rgba(0,0,0,0.1);">
            <div style="display: flex; flex-wrap: wrap; gap: 8px; justify-content: center;">
              ${['today', 'yesterday'].map(preset => {
                const active = preset === this._selectedPreset;
                return `<button class="history-preset-btn ${active ? 'active' : ''}" data-preset="${preset}" style="padding: 6px 10px; border: 1px solid ${active ? '#2b7bd3' : '#ddd'}; background: ${active ? '#2b7bd3' : 'white'}; color: ${active ? 'white' : '#333'}; border-radius: 999px; cursor: pointer; font-size: 12px;">${this.rangeLabel(preset)}</button>`;
              }).join('')}
            </div>
            <div style="display: flex; justify-content: center; align-items: center; gap: 10px; flex-wrap: wrap;">
              <label style="display: grid; gap: 4px; color: #666; font-size: 12px;">
                <span>Vælg dag</span>
                <input type="date" class="history-date-input" value="${this._selectedDate}" min="${this.minSelectableDate()}" max="${this.todayLocalIso()}" style="padding: 8px 10px; border: 1px solid #ddd; background: white; color: #333; border-radius: 6px;" />
              </label>
              <div style="font-size: 12px; color: #666;">${this.rangeLabel(this._selectedPreset)}: ${this.formatDateDa(this._selectedDate)}</div>
            </div>
            <div style="display: flex; justify-content: center; align-items: end; gap: 10px; flex-wrap: wrap;">
              <label style="display: grid; gap: 4px; color: #666; font-size: 12px;">
                <span>Fra tid</span>
                <select class="history-from-hour" style="padding: 8px 10px; border: 1px solid #ddd; background: white; color: #333; border-radius: 6px;">
                  ${Array.from({ length: 24 }, (_, hour) => `<option value="${hour}" ${hour === this._fromHour ? 'selected' : ''}>${this.formatHourLabel(hour)}</option>`).join('')}
                </select>
              </label>
              <label style="display: grid; gap: 4px; color: #666; font-size: 12px;">
                <span>Til tid</span>
                <select class="history-to-hour" style="padding: 8px 10px; border: 1px solid #ddd; background: white; color: #333; border-radius: 6px;">
                  ${Array.from({ length: 24 }, (_, index) => index + 1).map(hour => `<option value="${hour}" ${hour === this._toHour ? 'selected' : ''}>${hour === 24 ? '24:00' : this.formatHourLabel(hour)}</option>`).join('')}
                </select>
              </label>
              <div style="font-size: 12px; color: #666; min-width: 120px;">${this.formatHourLabel(this._fromHour)} → ${this._toHour === 24 ? '24:00' : this.formatHourLabel(this._toHour)}</div>
            </div>
          </div>
          ${isDefrostOnly ? `
          <div class="history-card__defrost-timeline" id="defrost-timeline-${this._config.sensor}" style="padding: 40px 16px; position: relative;">
            <div style="font-size: 13px; color: #666; margin-bottom: 12px; display: flex; align-items: center; gap: 8px;">
              <span>${this._config.sensor === 'aux_heating' ? 'Aux heating periods:' : 'Defrost periods:'}</span>
              <div style="display: flex; gap: 8px; align-items: center;">
                <div style="width: 16px; height: 12px; background: #03a9f4; border-radius: 2px;"></div>
                <span style="font-size: 12px;">Active</span>
              </div>
            </div>
            <div id="defrost-bars-${this._config.sensor}" style="height: 48px; background: #f5f5f5; border-radius: 4px; position: relative; overflow: hidden;"></div>
            <div id="bar-tooltip-${this._config.sensor}" style="position: absolute; background: rgba(0,0,0,0.85); color: white; padding: 8px 12px; border-radius: 4px; font-size: 12px; pointer-events: none; display: none; white-space: nowrap; z-index: 1000;"></div>
          </div>
          ` : `
          <div class="history-card__chart" style="height: 300px; padding: 16px; width: 500px; max-width: 500px;">
            <canvas id="history-chart-${this._config.sensor}"></canvas>
          </div>
          ${showDefrostTimeline ? `
          <div class="history-card__defrost-timeline" id="defrost-timeline-${this._config.sensor}" style="padding: 8px 16px; border-top: 1px solid rgba(0,0,0,0.1); position: relative;">
            <div style="font-size: 11px; color: #666; margin-bottom: 4px; display: flex; align-items: center; gap: 8px;">
              <span>Defrost periods:</span>
              <div style="display: flex; gap: 8px; align-items: center;">
                <div style="width: 16px; height: 12px; background: #03a9f4; border-radius: 2px;"></div>
                <span style="font-size: 10px;">Active</span>
              </div>
            </div>
            <div style="padding-left: 60px; padding-right: 12px;">
              <div id="defrost-bars-${this._config.sensor}" style="height: 24px; background: #f5f5f5; border-radius: 4px; position: relative; overflow: hidden;"></div>
            </div>
            <div id="bar-tooltip-${this._config.sensor}" style="position: absolute; background: rgba(0,0,0,0.85); color: white; padding: 8px 12px; border-radius: 4px; font-size: 12px; pointer-events: none; display: none; white-space: nowrap; z-index: 1000;"></div>
          </div>
          ` : ''}
          `}
        </div>
      </ha-card>
    `;
    
    this.querySelectorAll('.history-preset-btn').forEach(btn => {
      btn.onclick = () => this.setPreset(btn.dataset.preset);
    });

    const dateInput = this.querySelector('.history-date-input');
    if (dateInput) {
      dateInput.onchange = () => this.setSelectedDate(dateInput.value);
    }

    const fromHourInput = this.querySelector('.history-from-hour');
    if (fromHourInput) {
      fromHourInput.onchange = () => this.setFromHour(parseInt(fromHourInput.value, 10));
    }

    const toHourInput = this.querySelector('.history-to-hour');
    if (toHourInput) {
      toHourInput.onchange = () => this.setToHour(parseInt(toHourInput.value, 10));
    }
  }

  showNoData() {
    const canvas = this.querySelector("canvas");
    if (canvas) {
      canvas.parentElement.innerHTML = '<div style="padding: 40px; text-align: center; color: #999;">Ingen historik for den valgte dag endnu. Data gemmes hvert 60. sekund.</div>';
      return;
    }

    const timeline = this.querySelector(`#defrost-timeline-${this._config.sensor}`);
    if (timeline) {
      timeline.innerHTML = '<div style="padding: 20px; text-align: center; color: #999;">Ingen historik for den valgte dag endnu.</div>';
    }
  }

  showError(message) {
    const canvas = this.querySelector("canvas");
    if (canvas) {
      canvas.parentElement.innerHTML = `<div style="padding: 40px; text-align: center; color: #f44336;">Error: ${message}</div>`;
      return;
    }

    const timeline = this.querySelector(`#defrost-timeline-${this._config.sensor}`);
    if (timeline) {
      timeline.innerHTML = `<div style="padding: 20px; text-align: center; color: #f44336;">Error: ${message}</div>`;
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
          borderWidth: 1.5,
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
              text: `Tid (${this.formatDateDa(this._selectedDate)} ${this.formatHourLabel(this._fromHour)}-${this._toHour === 24 ? '24:00' : this.formatHourLabel(this._toHour)})`,
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
              callback: (v) => `${typeof v === 'number' ? v.toFixed(1) : v}°C`,
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
