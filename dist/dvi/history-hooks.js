export function bindHistoryHooks(container, entityMap, dispatcher) {
	Object.entries(entityMap).forEach(([key, entityId]) => {
		if (!entityId) return;
		const el = container.querySelector(`.diagram-label[data-key="${key}"]`);
		if (!el) return;
		el.classList.add("clickable");
		el.onclick = () => openMoreInfo(dispatcher, entityId);
	});
}

export function bindIconHooks(container, entityMap, dispatcher) {
	Object.entries(entityMap).forEach(([key, entityId]) => {
		if (!entityId) return;
		container.querySelectorAll(`[data-icon-key="${key}"]`).forEach((icon) => {
			icon.classList.add("clickable");
			icon.onclick = () => openMoreInfo(dispatcher, entityId);
		});
	});
}

function openMoreInfo(target, entityId) {
	target.dispatchEvent(
		new CustomEvent("hass-more-info", {
			detail: { entityId },
			bubbles: true,
			composed: true,
		}),
	);
}
