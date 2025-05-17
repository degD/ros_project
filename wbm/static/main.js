var map = L.map('map').setView([51.505, -0.09], 13);
L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {
    maxZoom: 19,
    attribution: '&copy; <a href="http://www.openstreetmap.org/copyright">OpenStreetMap</a>'
}).addTo(map);

const configData = JSON.parse(document.body.dataset.token);
const serviceAreaCenter = configData["service_area_center"];
const serviceAreaRadius = configData["service_area_radius"];

var circle = L.circle(serviceAreaCenter, {
    color: 'red',
    fillColor: '#f03',
    fillOpacity: 0.1,
    radius: serviceAreaRadius
}).addTo(map);



let area = null;
let drawStatus = 0;  // 0 = not drawing / drawn, 1 = drawing
let currentPoints = [];

const button = document.getElementById("draw-area-button");

button.addEventListener("click", () => {
    if (drawStatus === 0) {
        // Start drawing
        currentPoints = [];
        if (area) {
            map.removeLayer(area);
        }
        drawStatus = 1;
        button.textContent = "Complete";
    } else {
        // Finish drawing
        if (currentPoints.length >= 3) {
            area = L.polygon(currentPoints).addTo(map);
        }
        drawStatus = 0;
        button.textContent = "Draw Area";
    }
});

// Click map to add point when drawing
map.on("click", function (e) {
    if (drawStatus === 1) {
        const latlng = [e.latlng.lat, e.latlng.lng];
        currentPoints.push(latlng);

        // Draw a temp preview (polyline)
        if (area) {
            map.removeLayer(area);
        }
        area = L.polygon(currentPoints, { color: 'blue', dashArray: '5,5' }).addTo(map);
    }
});