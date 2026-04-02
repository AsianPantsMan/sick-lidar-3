import express from "express";
import cors from "cors";
import fs from "fs";
import path from "path";

const app = express();
app.use(express.json());

const PORT = Number(process.env.PORT) || 5050;
const allowedOrigins = (process.env.CORS_ORIGINS || "")
  .split(",")
  .map((origin) => origin.trim())
  .filter(Boolean);

app.use(
  cors({
    origin(origin, callback) {
      if (!origin || allowedOrigins.length === 0 || allowedOrigins.includes(origin)) {
        return callback(null, true);
      }
      return callback(new Error("Not allowed by CORS"));
    },
  })
);

const CSV_PATH = path.join(process.cwd(), "aisles.csv");// change save path to aisles.csv possibly

let aisles = [];

function writeCSV(points) {
  const rows = ["aisleId,type,x,y,createdAt"];
  for (const point of points) {
    rows.push(`${point.id},${point.type},${point.x},${point.y},${point.createdAt || ""}`);
  }
  fs.writeFileSync(CSV_PATH, `${rows.join("\n")}\n`);
}

// Ensure CSV file exists
function resetCSV() {
  writeCSV([]);
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
    createdAt,
  };

  aisles.push(point);

  // Append to CSV
  fs.appendFileSync(CSV_PATH, `${aisleId},${pointType},${x},${y},${createdAt}\n`);

  res.json({ success: true });
});

app.delete("/api/aisles", (req, res) => {
  aisles = [];
  writeCSV(aisles);
  res.json({ success: true });
});

app.listen(PORT, () => {
  console.log(`Backend running on port ${PORT}`);
});
