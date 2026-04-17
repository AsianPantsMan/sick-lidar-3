import { useEffect, useMemo, useRef, useState } from "react";
import { getDownloadURL, listAll, ref } from "firebase/storage";
import yaml from "js-yaml";
import { storage } from "./firebase/config";

function clamp(n, a, b) {
  return Math.max(a, Math.min(b, n));
}

function pixelToWorld({ px, py, imgH, resolution, origin }) {
  const originX = origin[0];
  const originY = origin[1];
  const mapPy = (imgH - 1) - py;

  return {
    x: originX + px * resolution,
    y: originY + mapPy * resolution,
  };
}

function worldToPixel({ x, y, imgH, resolution, origin }) {
  const originX = origin[0];
  const originY = origin[1];
  const px = (x - originX) / resolution;
  const mapPy = (y - originY) / resolution;
  const py = (imgH - 1) - mapPy;

  return { px, py };
}

const STEPS = ["start", "center", "end"];
const MAP_STORAGE_PATH = "slam_maps";

function labelForType(type) {
  if (type === "start") return "begin";
  if (type === "center") return "middle";
  return "end";
}

function incrementAisleId(id) {
  const match = String(id).trim().match(/^([A-Za-z]+)(\d+)$/);
  if (!match) return id;
  return `${match[1]}${Number.parseInt(match[2], 10) + 1}`;
}

function isPgmFile(name) {
  return String(name).toLowerCase().endsWith(".pgm");
}

function isYamlFile(name) {
  const lower = String(name).toLowerCase();
  return lower.endsWith(".yaml") || lower.endsWith(".yml");
}

function getParentPath(fullPath) {
  return String(fullPath).split("/").slice(0, -1).join("/");
}

function mapLabelForOption(path) {
  return path
    .replace(new RegExp(`^${MAP_STORAGE_PATH}/?`), "")
    .split("/")
    .filter(Boolean)
    .join(" / ");
}

async function collectStorageFiles(folderRef) {
  const snapshot = await listAll(folderRef);
  const nested = await Promise.all(snapshot.prefixes.map((prefixRef) => collectStorageFiles(prefixRef)));

  return [
    ...snapshot.items.map((item) => ({
      fullPath: item.fullPath,
      name: item.name,
    })),
    ...nested.flat(),
  ];
}

function buildMapFolderOptions(files) {
  const folderMap = files.reduce((groups, file) => {
    const folderPath = getParentPath(file.fullPath);
    const group = groups.get(folderPath) || {
      folderPath,
      label: mapLabelForOption(folderPath),
      pgmPath: null,
      yamlPath: null,
    };

    if (isPgmFile(file.name) && !group.pgmPath) {
      group.pgmPath = file.fullPath;
    }

    if (isYamlFile(file.name) && !group.yamlPath) {
      group.yamlPath = file.fullPath;
    }

    groups.set(folderPath, group);
    return groups;
  }, new Map());

  return Array.from(folderMap.values())
    .filter((group) => group.pgmPath && group.yamlPath)
    .sort((a, b) => a.label.localeCompare(b.label));
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

export default function MapAnnotator({ saved = [], refreshSaved, apiBaseUrl }) {
  const imgRef = useRef(null);

  if (!apiBaseUrl) {
    throw new Error("Missing VITE_API_URL.");
  }

  const [mapConfig, setMapConfig] = useState(null);
  const [mapOptions, setMapOptions] = useState([]);
  const [selectedMapFolder, setSelectedMapFolder] = useState("");
  const [mapsLoading, setMapsLoading] = useState(true);
  const [mapLoading, setMapLoading] = useState(false);
  const [mapError, setMapError] = useState(null);
  const [imgNatural, setImgNatural] = useState({ w: 0, h: 0 });
  const [lastClick, setLastClick] = useState(null);

  const [aisleId, setAisleId] = useState("A1");
  const [pointType, setPointType] = useState("start");
  const [completedTypes, setCompletedTypes] = useState(() => new Set());

  const savedTypesByAisle = useMemo(() => {
    return saved.reduce((groups, point) => {
      const aisleKey = String(point.id);
      const typeSet = groups.get(aisleKey) || new Set();
      typeSet.add(String(point.type));
      groups.set(aisleKey, typeSet);
      return groups;
    }, new Map());
  }, [saved]);

  const aislesWithAllPoints = useMemo(() => {
    return new Set(
      Array.from(savedTypesByAisle.entries())
        .filter(([, typeSet]) => STEPS.every((step) => typeSet.has(step)))
        .map(([aisleKey]) => aisleKey),
    );
  }, [savedTypesByAisle]);

  const existingSavedTypes = savedTypesByAisle.get(String(aisleId)) || new Set();

  function findFirstAvailableAisle(startAisleId) {
    let candidate = String(startAisleId);

    while (true) {
      const savedTypes = savedTypesByAisle.get(candidate) || new Set();
      const hasOpenSlot = STEPS.some((step) => !savedTypes.has(step));

      if (hasOpenSlot) {
        return candidate;
      }

      candidate = incrementAisleId(candidate);
    }
  }

  const availableTypes = useMemo(() => {
    return STEPS.filter((type) => !completedTypes.has(type) && !existingSavedTypes.has(type));
  }, [completedTypes, existingSavedTypes]);

  useEffect(() => {
    if (!availableTypes.includes(pointType)) {
      setPointType(availableTypes[0] ?? "start");
    }
  }, [availableTypes, pointType]);

  useEffect(() => {
    const firstAvailableAisle = findFirstAvailableAisle(aisleId);

    if (firstAvailableAisle === aisleId) {
      return;
    }

    setLastClick(null);
    setCompletedTypes(new Set());
    setPointType("start");
    setAisleId(firstAvailableAisle);
  }, [aisleId, savedTypesByAisle]);

  useEffect(() => {
    setMapConfig(null);
    setMapLoading(false);
    setLastClick(null);
  }, []);

  useEffect(() => {
    let active = true;

    async function loadMapOptions() {
      setMapsLoading(true);
      setMapError(null);

      try {
        const files = await collectStorageFiles(ref(storage, MAP_STORAGE_PATH));
        const options = buildMapFolderOptions(files);

        if (!active) return;

        setMapOptions(options);
        setSelectedMapFolder((current) => (options.some((option) => option.folderPath === current) ? current : ""));

        if (options.length === 0) {
          setMapError(`No folders under /${MAP_STORAGE_PATH} with both a .pgm and .yaml file were found.`);
        }
      } catch (err) {
        if (!active) return;
        console.error("Failed to load map options:", err);
        setMapError(String(err));
      } finally {
        if (active) setMapsLoading(false);
      }
    }

    loadMapOptions().catch(console.error);

    return () => {
      active = false;
    };
  }, []);

  const selectedMapOption = useMemo(
    () => mapOptions.find((option) => option.folderPath === selectedMapFolder) || null,
    [mapOptions, selectedMapFolder],
  );

  function handleMapSelection(nextFolderPath) {
    if (nextFolderPath === selectedMapFolder) return;

    setMapConfig(null);
    setImgNatural({ w: 0, h: 0 });
    setLastClick(null);
    setMapError(null);
    setSelectedMapFolder(nextFolderPath);

    if (!nextFolderPath) {
      setMapLoading(false);
      return;
    }

    setMapLoading(true);
  }

  useEffect(() => {
    if (!selectedMapOption) {
      setMapLoading(false);
      return;
    }

    let active = true;

    async function loadSelectedMapFromStorage() {
      setMapLoading(true);
      setMapError(null);

      try {
        const yamlUrl = await getDownloadURL(ref(storage, selectedMapOption.yamlPath));
        const yamlResponse = await fetch(`${apiBaseUrl}/api/storage-proxy`, {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ url: yamlUrl }),
        });

        if (!yamlResponse.ok) {
          throw new Error(`YAML fetch failed with HTTP ${yamlResponse.status}`);
        }

        const yamlText = await yamlResponse.text();
        const pgmUrl = await getDownloadURL(ref(storage, selectedMapOption.pgmPath));
        const pgmResponse = await fetch(`${apiBaseUrl}/api/storage-proxy`, {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ url: pgmUrl }),
        });

        if (!pgmResponse.ok) {
          throw new Error(`PGM fetch failed with HTTP ${pgmResponse.status}`);
        }

        const pgmBytes = new Uint8Array(await pgmResponse.arrayBuffer());
        const parsed = yaml.load(yamlText) || {};
        const pngDataUrl = parsePgmToDataUrl(pgmBytes);
        const resolution = Number(parsed.resolution);
        const origin = Array.isArray(parsed.origin) ? parsed.origin : null;

        if (!Number.isFinite(resolution) || !origin || origin.length < 2) {
          throw new Error(`Invalid YAML map metadata in ${selectedMapOption.yamlPath}`);
        }

        if (!active) return;

        setMapConfig({
          imageUrl: pngDataUrl,
          resolution,
          origin,
          sourcePath: selectedMapOption.folderPath,
        });
        setImgNatural({ w: 0, h: 0 });
        setLastClick(null);
      } catch (err) {
        if (!active) return;
        console.error("Failed to load selected storage map:", err);
        setMapError(String(err));
      } finally {
        if (active) setMapLoading(false);
      }
    }

    loadSelectedMapFromStorage().catch(console.error);

    return () => {
      active = false;
    };
  }, [selectedMapOption]);

  function handleImageLoad() {
    const img = imgRef.current;
    if (!img) return;
    setImgNatural({ w: img.naturalWidth, h: img.naturalHeight });
  }

  const plottedSavedPoints = useMemo(() => {
    if (!mapConfig || imgNatural.w === 0 || imgNatural.h === 0) {
      return [];
    }

    return saved
      .map((point) => {
        const { px, py } = worldToPixel({
          x: Number(point.x),
          y: Number(point.y),
          imgH: imgNatural.h,
          resolution: mapConfig.resolution,
          origin: mapConfig.origin,
        });

        return {
          ...point,
          px,
          py,
        };
      })
      .filter((point) => Number.isFinite(point.px) && Number.isFinite(point.py))
      .filter((point) => point.px >= 0 && point.py >= 0 && point.px <= imgNatural.w && point.py <= imgNatural.h);
  }, [saved, mapConfig, imgNatural.h, imgNatural.w]);

  function handleClick(e) {
    const img = imgRef.current;
    if (!img || !mapConfig || imgNatural.w === 0) return;

    const rect = img.getBoundingClientRect();
    const clickX = e.clientX - rect.left;
    const clickY = e.clientY - rect.top;
    const nx = clamp(clickX / rect.width, 0, 1);
    const ny = clamp(clickY / rect.height, 0, 1);
    const px = nx * imgNatural.w;
    const py = ny * imgNatural.h;

    const { x, y } = pixelToWorld({
      px,
      py,
      imgH: imgNatural.h,
      resolution: mapConfig.resolution,
      origin: mapConfig.origin,
    });

    setLastClick({
      px,
      py,
      worldX: x,
      worldY: y,
      markerLeft: clickX,
      markerTop: clickY,
    });
  }

  async function savePoint() {
    if (!lastClick) return;

    await fetch(`${apiBaseUrl}/api/aisles`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        aisleId,
        pointType,
        x: lastClick.worldX,
        y: lastClick.worldY,
      }),
    });

    await refreshSaved();

    setCompletedTypes((prev) => {
      const next = new Set(prev);
      next.add(pointType);
      return next;
    });

    if (pointType === "end") {
      setAisleId((prev) => incrementAisleId(prev));
      setCompletedTypes(new Set());
      setPointType("start");
    } else {
      const idx = STEPS.indexOf(pointType);
      const nextType = STEPS[idx + 1];
      if (nextType) setPointType(nextType);
    }

    setLastClick(null);
  }

  return (
    <div className="min-h-screen p-6 bg-gray-50 rounded-xl">
      <div className="max-w-6xl mx-auto space-y-6">
        <h1 className="text-3xl md:text-4xl font-black tracking-tight text-black">
          Retail Assistant Staff Interface
        </h1>

        <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
          <div className="lg:col-span-2 bg-white rounded-xl shadow p-4">
            <div className="flex items-baseline justify-between mb-3">
              <h2 className="text-xl font-semibold">Aisle Coordinate Selector</h2>
              <div className="text-sm text-gray-700">
                Aisle: <span className="font-mono font-semibold">{aisleId}</span>
              </div>
            </div>

            <div className="mb-3 rounded-xl border border-gray-200 bg-gray-50 p-3">
              <label className="block text-sm font-medium text-gray-700 mb-1">Map</label>
              <select
                value={selectedMapFolder}
                onChange={(e) => handleMapSelection(e.target.value)}
                className="w-full rounded-md border border-gray-300 bg-white px-3 py-2 text-sm"
                disabled={mapsLoading || mapOptions.length === 0}
              >
                <option value="">Select a map...</option>
                {mapOptions.map((option) => (
                  <option key={option.folderPath} value={option.folderPath}>
                    {option.label}
                  </option>
                ))}
              </select>
              <div className="mt-1 text-xs text-gray-500">
                {mapsLoading
                  ? "Loading maps from /slam_maps..."
                  : mapLoading && selectedMapFolder
                    ? `Loading: ${selectedMapFolder}`
                    : mapError
                      ? `Map status: ${mapError}`
                      : mapConfig?.sourcePath
                        ? `Active map: ${mapConfig.sourcePath}`
                        : "No map selected."}
              </div>
            </div>

            <div className="text-sm text-gray-600 mb-3">
              Click on the map to select the <span className="font-medium">{labelForType(pointType)}</span> point.
            </div>

            <div className="relative w-full select-none">
              {mapConfig ? (
                <img
                  key={mapConfig.sourcePath || mapConfig.imageUrl}
                  ref={imgRef}
                  src={mapConfig.imageUrl}
                  alt="Map"
                  onLoad={handleImageLoad}
                  onClick={handleClick}
                  className="block w-full h-auto border rounded-lg cursor-crosshair"
                  draggable={false}
                />
              ) : (
                <div className="w-full h-80 rounded-lg border border-dashed border-gray-300 bg-gray-50 text-sm text-gray-500 flex items-center justify-center">
                  {mapLoading ? "Loading selected map..." : "Select a map to display it."}
                </div>
              )}

              {plottedSavedPoints.map((point) => (
                <div
                  key={`${point.id}-${point.type}-${point.x}-${point.y}`}
                  className="absolute -translate-x-1/2 -translate-y-1/2"
                  style={{
                    left: `${(point.px / imgNatural.w) * 100}%`,
                    top: `${(point.py / imgNatural.h) * 100}%`,
                  }}
                  title={`${point.id} - ${point.type}`}
                >
                  {point.type === "center" && aislesWithAllPoints.has(String(point.id)) && (
                    <div className="absolute left-1/2 -top-5 -translate-x-1/2 rounded-full bg-white/90 px-2 py-0.5 text-[10px] font-semibold tracking-wide text-gray-900 shadow-sm ring-1 ring-gray-200 whitespace-nowrap">
                      {point.id}
                    </div>
                  )}
                  <div
                    className={`w-3 h-3 rounded-full border-2 border-white shadow-md ${
                      point.type === "start"
                        ? "bg-emerald-500"
                        : point.type === "center"
                          ? "bg-amber-500"
                          : "bg-red-600"
                    }`}
                  />
                </div>
              ))}

              {lastClick && mapConfig && (
                <div
                  className="absolute w-3 h-3 rounded-full bg-black border-2 border-white -translate-x-1/2 -translate-y-1/2"
                  style={{
                    left: `${lastClick.markerLeft}px`,
                    top: `${lastClick.markerTop}px`,
                  }}
                  title="Selected point"
                />
              )}
            </div>
          </div>

          <div className="bg-white rounded-xl shadow p-4">
            <h3 className="text-lg font-semibold mb-3">Save Point</h3>

            <div className="mb-3">
              <div className="block text-sm text-gray-700 mb-1">Current Aisle</div>
              <div className="font-mono border rounded-md px-3 py-2 bg-gray-50">
                {aisleId}
              </div>
            </div>

            <label className="block text-sm text-gray-700 mb-1">Point Type</label>
            <select
              value={pointType}
              onChange={(e) => setPointType(e.target.value)}
              className="w-full border rounded-md px-3 py-2 mb-2"
              disabled={availableTypes.length === 0}
            >
              {availableTypes.map((type) => (
                <option key={type} value={type}>
                  {labelForType(type)}
                </option>
              ))}
            </select>

            <div className="text-xs text-gray-500 mb-4">
              Remaining for {aisleId}: <span className="font-mono">{availableTypes.map(labelForType).join(", ") || "none"}</span>
            </div>

            <button
              onClick={savePoint}
              disabled={!lastClick || availableTypes.length === 0}
              className="w-full bg-red-900 text-white rounded-xl py-2 disabled:opacity-40 hover:bg-red-950 transition-colors"
            >
              Save Selected Point
            </button>

            <hr className="my-4" />

            <div className="space-y-3 text-sm">
              <div className="rounded-xl border border-gray-200 bg-gray-50 p-3">
                <div className="text-gray-600">Resolution</div>
                <div className="font-mono">
                  {mapConfig ? `${mapConfig.resolution} m/pixel` : "--"}
                </div>
              </div>

              <div className="rounded-xl border border-gray-200 bg-gray-50 p-3">
                <div className="text-gray-600">Origin</div>
                <div className="font-mono">
                  {mapConfig
                    ? `[${mapConfig.origin[0]}, ${mapConfig.origin[1]}, ${mapConfig.origin[2]}]`
                    : "--"}
                </div>
              </div>

              <div className="rounded-xl border border-red-200 bg-red-50 p-3">
                <div className="text-red-900">Currently Selected Point</div>
                {lastClick ? (
                  <div className="mt-1 font-mono text-red-950">
                    pixel=({lastClick.px.toFixed(1)}, {lastClick.py.toFixed(1)})
                    <br />
                    world=({lastClick.worldX.toFixed(3)}, {lastClick.worldY.toFixed(3)}) meters
                  </div>
                ) : (
                  <div className="mt-1 text-red-800">No point selected yet.</div>
                )}
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
