import express from "express";
import cors from "cors";
import fs from "fs";
import path from "path";
import { fileURLToPath } from "url";

const app = express();
app.use(express.json());

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);

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

const CSV_PATH = path.join(__dirname, "aisles.csv");
const CSV_HEADER = "aisleId,type,x,y";
const LEGACY_CSV_HEADER = "aisleId,type,x,y,createdAt";

let aisles = [];

function parseCsvLine(line) {
  return line.split(",").map((part) => part.trim());
}

function toNumberOrNull(value) {
  const n = Number(value);
  return Number.isFinite(n) ? n : null;
}

function loadAislesFromCSV() {
  if (!fs.existsSync(CSV_PATH)) {
    writeCSV([]);
    return [];
  }

  const raw = fs.readFileSync(CSV_PATH, "utf8").trim();
  if (!raw) return [];

  const lines = raw.split("\n").filter(Boolean);
  if (lines.length === 0) return [];

  const header = lines[0].trim();
  const isLegacyHeader = header === LEGACY_CSV_HEADER;
  if (header !== CSV_HEADER && !isLegacyHeader) {
    const backupPath = path.join(__dirname, `aisles.invalid.${Date.now()}.bak`);
    fs.writeFileSync(backupPath, `${raw}\n`);
    writeCSV([]);
    console.warn(
      `Invalid CSV format detected at ${CSV_PATH}. Backed up malformed data to ${backupPath} and reset CSV.`
    );
    return [];
  }

  const [, ...dataLines] = lines;

  const parsedAisles = dataLines
    .map((line) => {
      const [aisleId, type, x, y] = parseCsvLine(line);

      if (!aisleId || !type) return null;

      return {
        id: aisleId,
        type,
        x: toNumberOrNull(x),
        y: toNumberOrNull(y),
      };
    })
    .filter((point) => point && point.x !== null && point.y !== null);

  if (isLegacyHeader) {
    writeCSV(parsedAisles);
    console.log("Migrated legacy aisles CSV format to remove createdAt column.");
  }

  return parsedAisles;
}

function writeCSV(points) {
  const rows = [CSV_HEADER];
  for (const point of points) {
    rows.push(`${point.id},${point.type},${point.x},${point.y}`);
  }
  fs.writeFileSync(CSV_PATH, `${rows.join("\n")}\n`);
}

function appendPointToCSV(point) {
  const row = `${point.id},${point.type},${point.x},${point.y}\n`;
  fs.appendFileSync(CSV_PATH, row);
}

aisles = loadAislesFromCSV();
console.log(`Loaded ${aisles.length} saved coordinate(s) from CSV.`);

app.get("/api/aisles", (req, res) => {
  res.json({ aisles });
});

app.get("/api/aisles/all", (req, res) => {
  res.json({
    count: aisles.length,
    coordinates: aisles,
  });
});

app.post("/api/aisles", (req, res) => {
  const { aisleId, pointType, x, y } = req.body;

  const normalizedAisleId = String(aisleId || "").trim();
  const normalizedPointType = String(pointType || "").trim().toLowerCase();
  const xNum = Number(x);
  const yNum = Number(y);

  if (!normalizedAisleId || !normalizedPointType) {
    return res.status(400).json({
      success: false,
      error: "aisleId and pointType are required.",
    });
  }

  if (!Number.isFinite(xNum) || !Number.isFinite(yNum)) {
    return res.status(400).json({
      success: false,
      error: "x and y must be valid numbers.",
    });
  }

  const point = {
    id: normalizedAisleId,
    type: normalizedPointType,
    x: xNum,
    y: yNum,
  };

  aisles.push(point);

  appendPointToCSV(point);

  res.status(201).json({ success: true, point });
});

app.delete("/api/aisles", (req, res) => {
  aisles = [];
  writeCSV(aisles);
  res.json({ success: true });
});

app.listen(PORT, () => {
  console.log(`Backend running on port ${PORT}`);
});
