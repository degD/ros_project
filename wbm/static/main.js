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



let area = null;               // Final completed polygon
let tempPreview = null;        // Temporary polygon while drawing
let drawStatus = 0;            // 0 = not drawing, 1 = drawing
let currentPoints = [];

const button = document.getElementById("draw-area-button");

button.addEventListener("click", () => {
    if (drawStatus === 0) {
        // Start drawing
        currentPoints = [];
        if (area) {
            map.removeLayer(area);
            area = null;
        }
        if (tempPreview) {
            map.removeLayer(tempPreview);
            tempPreview = null;
        }
        drawStatus = 1;
        button.textContent = "Complete";
    } else {
        // Complete drawing
        if (currentPoints.length >= 3) {
            if (tempPreview) {
                map.removeLayer(tempPreview);
                tempPreview = null;
            }
            area = L.polygon(currentPoints, { color: 'blue' }).addTo(map);
        }
        drawStatus = 0;
        button.textContent = "Draw Area";
    }
});

map.on("click", function (e) {
    if (drawStatus === 1) {
        currentPoints.push([e.latlng.lat, e.latlng.lng]);

        // Remove old preview before adding new one
        if (tempPreview) {
            map.removeLayer(tempPreview);
        }
        tempPreview = L.polygon(currentPoints, {
            color: 'blue',
            dashArray: '5, 5',
            fillOpacity: 0.1
        }).addTo(map);
    }
});
