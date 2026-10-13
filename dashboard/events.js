"use strict";

const historyList = document.getElementById("event-list");
const historyCount = document.getElementById("event-count");

function historyMessage(message) {
  historyList.replaceChildren();
  const li = document.createElement("li");
  li.textContent = message;
  historyList.appendChild(li);
}

async function refreshHistory() {
  try {
    const response = await fetch("/api/events", {
      cache: "no-store"
    });
    if (!response.ok) throw Error("HTTP " + response.status);

    const data = await response.json();
    if (!Array.isArray(data.events)) {
      throw Error("Formato eventi non valido");
    }

    const events = data.events.slice(0, 15);
    historyCount.textContent =
      "Ultimi " + events.length + " eventi (massimo 15 visualizzati)";

    historyList.replaceChildren();

    if (!events.length) {
      historyMessage("Nessun evento registrato.");
      return;
    }

    for (const event of events) {
      const li = document.createElement("li");
      li.className = "event-row " +
        (event.type === "FAULT" ? "fault" : "recovery");

      const title = document.createElement("strong");
      title.textContent = event.type + " · " + event.component;

      const detail = document.createElement("span");
      const timestamp = Date.parse(event.timestamp_utc);
      const when = Number.isFinite(timestamp)
        ? new Date(timestamp).toLocaleString("it-IT", {
            timeZone: "Europe/Rome"
          })
        : "Orario non disponibile";

      const duration = typeof event.duration_seconds === "number"
        ? " · Durata: " + event.duration_seconds.toFixed(1) + " s"
        : "";

      detail.textContent = when + duration;

      const description = document.createElement("p");
      description.textContent = event.message || "";

      li.append(title, detail, description);
      historyList.appendChild(li);
    }
  } catch (error) {
    historyCount.textContent = "Storico non disponibile";
    historyMessage("Errore API eventi: " + error.message);
  }
}

refreshHistory();
setInterval(refreshHistory, 5000);
