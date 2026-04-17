import { useEffect, useState } from "react";
import { getDownloadURL, listAll, ref } from "firebase/storage";
import yaml from "js-yaml";
import { storage } from "../firebase/config";

const SELECTED_MAP_STORAGE_KEY = "selectedMapFolder";
const SELECTED_MAP_CONFIG_STORAGE_KEY = "selectedMapConfig";

function isPgmFile(name) {
  return String(name).toLowerCase().endsWith(".pgm");
}

function isYamlFile(name) {
  const lower = String(name).toLowerCase();
  return lower.endsWith(".yaml") || lower.endsWith(".yml");
}

function parsePgmToDataUrl(bytes) {
  let token = "";
  const tokens = [];
  let index = 0;

  const isWhitespace = (value) => value === 9 || value === 10 || value === 13 || value === 32 || value === 12;

  while (index < bytes.length && tokens.length < 4) {
    const current = bytes[index];

    if (current === 35) {
      while (index < bytes.length && bytes[index] !== 10) {
        index += 1;
      }
      continue;
    }

    if (isWhitespace(current)) {
      if (token) {
        tokens.push(token);
        token = "";
      }
      index += 1;
      continue;
    }

    token += String.fromCharCode(current);
    index += 1;
  }

  if (token && tokens.length < 4) {
    tokens.push(token);
  }

  const [magic, widthText, heightText, maxValueText] = tokens;
  const width = Number(widthText);
  const height = Number(heightText);
  const maxValue = Number(maxValueText);

  if (!magic || !Number.isFinite(width) || !Number.isFinite(height) || !Number.isFinite(maxValue)) {
    throw new Error("Invalid PGM header.");
  }

  while (index < bytes.length && isWhitespace(bytes[index])) {
    index += 1;
  }

  const imageData = new ImageData(width, height);

  if (magic === "P5") {
    const bytesPerSample = maxValue > 255 ? 2 : 1;
    const pixelCount = width * height;

    for (let pixelIndex = 0; pixelIndex < pixelCount; pixelIndex += 1) {
      let gray = 0;

      if (bytesPerSample === 1) {
        gray = bytes[index + pixelIndex] ?? 0;
      } else {
        const byteOffset = index + pixelIndex * 2;
        const high = bytes[byteOffset] ?? 0;
        const low = bytes[byteOffset + 1] ?? 0;
        gray = (high << 8) | low;
      }

      const scaled = maxValue > 0 ? Math.round((gray / maxValue) * 255) : gray;
      const targetIndex = pixelIndex * 4;
      imageData.data[targetIndex] = scaled;
      imageData.data[targetIndex + 1] = scaled;
      imageData.data[targetIndex + 2] = scaled;
      imageData.data[targetIndex + 3] = 255;
    }
  } else if (magic === "P2") {
    const pixelValues = new TextDecoder().decode(bytes.slice(index)).split(/\s+/).filter(Boolean).map(Number);

    for (let pixelIndex = 0; pixelIndex < width * height; pixelIndex += 1) {
      const gray = pixelValues[pixelIndex] ?? 0;
      const scaled = maxValue > 0 ? Math.round((gray / maxValue) * 255) : gray;
      const targetIndex = pixelIndex * 4;
      imageData.data[targetIndex] = scaled;
      imageData.data[targetIndex + 1] = scaled;
      imageData.data[targetIndex + 2] = scaled;
      imageData.data[targetIndex + 3] = 255;
    }
  } else {
    throw new Error(`Unsupported PGM format: ${magic}`);
  }

  const canvas = document.createElement("canvas");
  canvas.width = width;
  canvas.height = height;
  const context = canvas.getContext("2d");

  if (!context) {
    throw new Error("Unable to render map canvas.");
  }

  context.putImageData(imageData, 0, 0);
  return canvas.toDataURL("image/png");
}

function buildMapConfigFromFolder(folderPath, yamlPath, yamlText, pgmBytes) {
  const parsed = yaml.load(yamlText) || {};
  const pngDataUrl = parsePgmToDataUrl(pgmBytes);
  const resolution = Number(parsed.resolution);
  const origin = Array.isArray(parsed.origin) ? parsed.origin : null;

  if (!Number.isFinite(resolution) || !origin || origin.length < 2) {
    throw new Error(`Invalid YAML map metadata in ${yamlPath}`);
  }

  return {
    imageUrl: pngDataUrl,
    resolution,
    origin,
    sourcePath: folderPath,
  };
}

export default function useSelectedMap(apiBaseUrl) {
  const [selectedMapFolder, setSelectedMapFolder] = useState(() => {
    if (typeof window === "undefined") return "";
    return window.localStorage.getItem(SELECTED_MAP_STORAGE_KEY) || "";
  });
  const [mapConfig, setMapConfig] = useState(() => {
    if (typeof window === "undefined") return null;

    try {
      const cached = window.localStorage.getItem(SELECTED_MAP_CONFIG_STORAGE_KEY);
      if (!cached) return null;

      const parsed = JSON.parse(cached);
      return parsed?.sourcePath ? parsed : null;
    } catch {
      return null;
    }
  });
  const [mapLoading, setMapLoading] = useState(false);
  const [mapError, setMapError] = useState(null);

  useEffect(() => {
    if (typeof window === "undefined") return;

    if (selectedMapFolder) {
      window.localStorage.setItem(SELECTED_MAP_STORAGE_KEY, selectedMapFolder);
    } else {
      window.localStorage.removeItem(SELECTED_MAP_STORAGE_KEY);
    }
  }, [selectedMapFolder]);

  useEffect(() => {
    if (typeof window === "undefined") return;

    if (mapConfig) {
      window.localStorage.setItem(SELECTED_MAP_CONFIG_STORAGE_KEY, JSON.stringify(mapConfig));
    } else if (!selectedMapFolder) {
      window.localStorage.removeItem(SELECTED_MAP_CONFIG_STORAGE_KEY);
    }
  }, [mapConfig, selectedMapFolder]);

  useEffect(() => {
    if (!apiBaseUrl) {
      setMapConfig(null);
      setMapLoading(false);
      setMapError("Missing VITE_API_URL.");
      return;
    }

    if (!selectedMapFolder) {
      setMapConfig(null);
      setMapLoading(false);
      setMapError(null);
      return;
    }

    let active = true;

    async function loadSelectedMap() {
      setMapLoading(true);
      setMapError(null);

      try {
        const folderRef = ref(storage, selectedMapFolder);
        const snapshot = await listAll(folderRef);
        const yamlItem = snapshot.items.find((item) => isYamlFile(item.name));
        const pgmItem = snapshot.items.find((item) => isPgmFile(item.name));

        if (!yamlItem || !pgmItem) {
          throw new Error(`Map folder ${selectedMapFolder} is missing a .yaml or .pgm file.`);
        }

        const [yamlUrl, pgmUrl] = await Promise.all([
          getDownloadURL(ref(storage, yamlItem.fullPath)),
          getDownloadURL(ref(storage, pgmItem.fullPath)),
        ]);

        const [yamlResponse, pgmResponse] = await Promise.all([
          fetch(`${apiBaseUrl}/api/storage-proxy`, {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ url: yamlUrl }),
          }),
          fetch(`${apiBaseUrl}/api/storage-proxy`, {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ url: pgmUrl }),
          }),
        ]);

        if (!yamlResponse.ok) {
          throw new Error(`YAML fetch failed with HTTP ${yamlResponse.status}`);
        }

        if (!pgmResponse.ok) {
          throw new Error(`PGM fetch failed with HTTP ${pgmResponse.status}`);
        }

        const [yamlText, pgmBytes] = await Promise.all([
          yamlResponse.text(),
          pgmResponse.arrayBuffer(),
        ]);

        const config = buildMapConfigFromFolder(
          selectedMapFolder,
          yamlItem.fullPath,
          yamlText,
          new Uint8Array(pgmBytes),
        );

        if (!active) return;
        setMapConfig(config);
      } catch (err) {
        if (!active) return;
        setMapConfig(null);
        setMapError(String(err));
      } finally {
        if (active) setMapLoading(false);
      }
    }

    loadSelectedMap().catch((err) => {
      if (!active) return;
      setMapConfig(null);
      setMapError(String(err));
      setMapLoading(false);
    });

    return () => {
      active = false;
    };
  }, [apiBaseUrl, selectedMapFolder]);

  return {
    selectedMapFolder,
    setSelectedMapFolder,
    mapConfig,
    mapLoading,
    mapError,
  };
}
