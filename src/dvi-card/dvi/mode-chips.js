export function wireModeChips(container, hass, chipGroups = {}) {
	container.querySelectorAll(".popup-chip").forEach((chip) => {
		const type = chip.dataset.popup;
		const group = chipGroups[type];
		if (!group || !Array.isArray(group.entities) || !group.entities.length) return;

		chip.onclick = () => {
			hass.callService("browser_mod", "popup", {
				title: group.title,
				content: {
					type: "entities",
					entities: group.entities,
				},
			});
		};
	});

	container.querySelectorAll("[data-open-confirm]").forEach((chip) => {
		chip.onclick = async () => {
			chip.classList.add("mode-chip--open-request-pending");
			try {
				if (hass?.connection) {
					await hass.callService("mqtt", "publish", {
						topic: "dvi/command/open_request",
						payload: "confirm",
					});
				} else {
					await fetch("/api/open_request/confirm", { method: "POST" });
				}
			} finally {
				chip.classList.remove("mode-chip--open-request-pending");
				if (hass?.refresh) {
					await hass.refresh();
				}
			}
		};
	});

	container.querySelectorAll("[data-open-close]").forEach((chip) => {
		chip.onclick = async () => {
			try {
				if (hass?.connection) {
					await hass.callService("mqtt", "publish", {
						topic: "dvi/command/open_request",
						payload: "close",
					});
				} else {
					await fetch("/api/open_request/close", { method: "POST" });
				}
			} finally {
				if (hass?.refresh) {
					await hass.refresh();
				}
				if (!hass?.connection) {
					try {
						window.close();
					} catch (e) {
						return;
					}
				}
			}
		};
	});
}
