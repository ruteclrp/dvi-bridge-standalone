(function () {
  const origAttachShadow = Element.prototype.attachShadow;
  Element.prototype.attachShadow = function (init) {
    const root = origAttachShadow.call(this, init);
    const style = document.createElement("style");
    style.textContent = `
      :host {
        --disabled-text-color: #999;
        --primary-text-color: #000 !important;
        --chip-text-color: #000 !important;
        --chip-active-text: #000 !important;
        --state-climate-heat-color: #ff9800;
        --state-water-heater-heat-color: #ff9800;
        --warning-color: #fdd835;
        --accent-color: #03a9f4;
      }
      ha-icon { color: inherit; }
      .diagram-icon, .diagram-element { color: inherit; }
    `;
    root.appendChild(style);

    const updateChipLabels = () => {
      root.querySelectorAll(".chip-label, .chip-value").forEach(el => {
        const chip = el.closest(".mode-chip");
        if (chip && chip.classList.contains("mode-chip--inactive")) {
          el.style.color = "#999";
        } else {
          el.style.color = "#000";
        }
      });
    };

    const observer = new MutationObserver(updateChipLabels);
    observer.observe(root, {
      childList: true,
      subtree: true,
      attributes: true,
      attributeFilter: ["class"]
    });

    setTimeout(updateChipLabels, 0);
    return root;
  };
})();
