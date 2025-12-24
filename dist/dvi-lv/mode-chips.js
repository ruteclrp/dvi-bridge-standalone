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
}
