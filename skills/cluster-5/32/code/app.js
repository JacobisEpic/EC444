const fs = require('fs');
const path = require('path');
const Db = require('tingodb')().Db;

const dbPath = path.join(__dirname, 'data');
if (!fs.existsSync(dbPath)) fs.mkdirSync(dbPath);

const db = new Db(dbPath, {});
const sensorData = db.collection("sensor_data");
fs.readFile('smoke.txt', 'utf8', (err, data) => {
    if (err) throw err;
    const lines = data.split('\n');
    lines.forEach((line, index) => {
        if (index > 0 && line.trim() !== '') {
            const [time, id, smoke, temp] = line.split('\t').map(v => v.trim());
            sensorData.insert({
                time: parseInt(time),
                sensor_id: parseInt(id),
                smoke: smoke === '1',
                temperature: parseFloat(temp)
            });
        }
    });
});

sensorData.find({ sensor_id: 1, smoke: true }).toArray((err, docs) => {
    if (err) throw err;
    console.log('Instances where sensor ID 1 detected smoke:');
    docs.forEach(doc => {
        console.log(`Time: ${doc.time}, Temperature: ${doc.temperature}°C`);
    });
});
