function updateSliderBackground(slider) {
    const min = parseFloat(slider.min || 0);
    const max = parseFloat(slider.max || 100);
    const value = parseFloat(slider.value || 0);
    const percent = ((value - min) / (max - min)) * 100;

    slider.style.background = `linear-gradient(to right,
        var(--accent-2) 0%,
        var(--accent-2) ${percent}%,
        #3a3a3a ${percent}%,
        #3a3a3a 100%)`;
}

document.addEventListener("DOMContentLoaded", () => {
    document.querySelectorAll(".slider").forEach(slider => {
        updateSliderBackground(slider);
        slider.addEventListener("input", () => updateSliderBackground(slider));
    });
});

async function refreshStats() {
    try {
        const res = await fetch("/api/stats");
        const data = await res.json();

        document.getElementById("fps-stat").textContent = data.fps;
        document.getElementById("temp-stat").textContent = `${data.temperature}°C`;
        document.getElementById("latency-stat").textContent = `${data.latency}ms`;
    } catch (err) {
        console.error("Failed to fetch stats:", err);
    }
}

setInterval(refreshStats, 1000);
refreshStats();

function getSettingValue(el) {
    if (el.type === "checkbox") return el.checked;
    if (el.type === "number" || el.type === "range") return Number(el.value);
    return el.value;
}

function setElementValue(el, value) {
    if (el.type === "checkbox") {
        el.checked = Boolean(value);
    } else {
        el.value = value;
    }

    if (el.classList.contains("slider")) {
        updateSliderBackground(el);
    }
}

function syncSettingElements(settingName, value, sourceElement = null) {
    document.querySelectorAll(`[data-setting="${settingName}"]`).forEach(el => {
        if (el !== sourceElement) {
            setElementValue(el, value);
        }
    });
}

async function sendSettingUpdate(name, value) {
    try {
        const res = await fetch("/api/settings", {
            method: "POST",
            headers: {
                "Content-Type": "application/json"
            },
            body: JSON.stringify({ name, value })
        });

        const data = await res.json();
        if (!data.ok) {
            console.error("Setting update failed:", data);
        }
    } catch (err) {
        console.error("Error sending setting update:", err);
    }
}

const debounceTimers = new Map();

function debounceSetting(name, callback, delay = 120) {
    if (debounceTimers.has(name)) {
        clearTimeout(debounceTimers.get(name));
    }
    debounceTimers.set(name, setTimeout(callback, delay));
}

function updateSliderBackground(slider) {
    const min = parseFloat(slider.min || 0);
    const max = parseFloat(slider.max || 100);
    const value = parseFloat(slider.value || 0);
    const percent = ((value - min) / (max - min)) * 100;

    slider.style.background = `linear-gradient(to right,
        var(--accent-2) 0%,
        var(--accent-2) ${percent}%,
        #3a3a3a ${percent}%,
        #3a3a3a 100%)`;
}

document.addEventListener("DOMContentLoaded", () => {
    document.querySelectorAll('.slider').forEach(updateSliderBackground);

    document.addEventListener("input", (e) => {
        const el = e.target.closest("[data-setting]");
        if (!el) return;

        const name = el.dataset.setting;
        const value = getSettingValue(el);

        syncSettingElements(name, value, el);

        debounceSetting(name, () => sendSettingUpdate(name, value));
    });

    document.addEventListener("change", (e) => {
        const el = e.target.closest("[data-setting]");
        if (!el) return;

        if (el.matches('select, input[type="checkbox"]')) {
            const name = el.dataset.setting;
            const value = getSettingValue(el);
            syncSettingElements(name, value, el);
            sendSettingUpdate(name, value);
        }
    });
});