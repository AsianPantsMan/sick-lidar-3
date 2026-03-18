import express from "express";
import cors from "cors";
import fs from "fs";
import path from "path";

const app = express();
app.use(cors());
app.use(express.json());

const PORT = 5050;
const CSV_PATH = path.join(process.cwd(), "aisles.csv");// change save path to aisles.csv possibly

let aisles = [];

// Ensure CSV file exists
function resetCSV() {
  fs.writeFileSync(CSV_PATH, "aisleId,type,x,y,createdAt\n");
  console.log("CSV reset on startup.");
}

resetCSV();

app.get("/api/aisles", (req, res) => {
  res.json({ aisles });
});

app.post("/api/aisles", (req, res) => {
  const { aisleId, pointType, x, y } = req.body;

  const createdAt = new Date().toISOString();

  const point = {
    id: aisleId,
    type: pointType,
    x,
    y,
  };

  aisles.push(point);

  // Append to CSV
  const row = `${aisleId},${pointType},${x},${y}\n`;
  fs.appendFileSync(CSV_PATH, row);

  res.json({ success: true });
});

app.delete("/api/aisles", (req, res) => {
  aisles = [];
  fs.writeFileSync(CSV_PATH, "aisleId,type,x,y,createdAt\n");
  res.json({ success: true });
});

app.listen(PORT, () => {
  console.log(`Backend running on http://localhost:${PORT}`);
});
