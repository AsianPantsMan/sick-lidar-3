import { useEffect, useMemo, useRef, useState } from "react";
import yaml from "js-yaml";

function clamp(n, a, b) {
  return Math.max(a, Math.min(b, n));
}

/**
 * Convert image click pixel (top-left origin) to ROS map world coords (meters).
 * Assumes origin yaw is 0.
 */
function pixelToWorld({ px, py, imgW, imgH, resolution, origin }) {
  const originX = origin[0];
  const originY = origin[1];

  // Flip Y because image y=0 is top, but map y=0 is bottom
  const mapPy = (imgH - 1) - py;

  const x = originX + px * resolution;
  const y = originY + mapPy * resolution;

  return { x, y };
}

function worldToPixel({ x, y, imgW, imgH, resolution, origin }) {
  const originX = origin[0];
  const originY = origin[1];

  const px = (x - originX) / resolution;
  const mapPy = (y - originY) / resolution;
  const py = (imgH - 1) - mapPy;

  return { px, py, mapPy, imgW, imgH };
}

// Internal canonical types (what you store in backend)
const STEPS = ["start", "center", "end"];

// UI labels
function labelForType(t) {
  if (t === "start") return "begin";
  if (t === "center") return "middle";
  return "end";
}

function incrementAisleId(id) {
  // supports A1, A12, B3, etc.
  const m = String(id).trim().match(/^([A-Za-z]+)(\d+)$/);
  if (!m) return id; // if format is unexpected, don't change it
  const prefix = m[1];
  const num = parseInt(m[2], 10);
  return `${prefix}${num + 1}`;
}

export default function MapAnnotator({ saved = [], refreshSaved, apiBaseUrl }) {
  const imgRef = useRef(null);

  if (!apiBaseUrl) {
    throw new Error("Missing VITE_API_URL.");
  }

  const [mapConfig, setMapConfig] = useState(null);
  const [imgNatural, setImgNatural] = useState({ w: 0, h: 0 });

  const [lastClick, setLastClick] = useState(null);

  // Wizard state lives here
  const [aisleId, setAisleId] = useState("A1");
  const [pointType, setPointType] = useState("start"); // start -> center -> end
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
    return STEPS.filter((t) => !completedTypes.has(t) && !existingSavedTypes.has(t));
  }, [completedTypes, existingSavedTypes]);

  // Make sure dropdown selection is always valid
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

  // Load YAML map config once
  useEffect(() => {
    async function loadYaml() {
      const response = await fetch("/maps/demo_map.map.yaml");
      const text = await response.text();
      const parsed = yaml.load(text);

      setMapConfig({
        imageUrl: "/maps/demo_map.map.png",
        resolution: Number(parsed.resolution),
        origin: parsed.origin,
      });
    }

    loadYaml().catch(console.error);
  }, []);

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
          imgW: imgNatural.w,
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

    // click in DISPLAY coords
    const clickX = e.clientX - rect.left;
    const clickY = e.clientY - rect.top;

    // normalize to [0..1] then scale to ORIGINAL pixel coords
    const nx = clamp(clickX / rect.width, 0, 1);
    const ny = clamp(clickY / rect.height, 0, 1);

    const px = nx * imgNatural.w;
    const py = ny * imgNatural.h;

    const { x, y } = pixelToWorld({
      px,
      py,
      imgW: imgNatural.w,
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
        pointType, // stored as start/center/end
        x: lastClick.worldX,
        y: lastClick.worldY,
      }),
    });

    await refreshSaved();

    //  remove this item from dropdown for current aisle
    setCompletedTypes((prev) => {
      const next = new Set(prev);
      next.add(pointType);
      return next;
    });

    // auto-advance the wizard
    if (pointType === "end") {
      // finished the aisle → next aisle, reset types
      setAisleId((prev) => incrementAisleId(prev));
      setCompletedTypes(new Set());
      setPointType("start");
    } else {
      // go to next point type in the sequence
      const idx = STEPS.indexOf(pointType);
      const nextType = STEPS[idx + 1];
      if (nextType) setPointType(nextType);
    }

    // Require a new click for the next step
    setLastClick(null);
  }

  if (!mapConfig) {
    return <div className="p-6">Loading map configuration...</div>;
  }

  return (
    <div className="min-h-screen p-6 bg-gray-50 rounded-xl">
      <div className="max-w-6xl mx-auto space-y-6">
        <div className="bg-white rounded-xl shadow p-4">
          <h1 className="text-3xl md:text-4xl font-black tracking-tight text-black">
            Retail Assistant Staff Interface
          </h1>
        </div>

        <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
        {/* Map panel */}
        <div className="lg:col-span-2 bg-white rounded-xl shadow p-4">
          <div className="flex items-baseline justify-between mb-3">
            <h2 className="text-xl font-semibold">Aisle Coordinate Selector</h2>
            <div className="text-sm text-gray-700">
              Aisle: <span className="font-mono font-semibold">{aisleId}</span>
            </div>
          </div>

          <div className="text-sm text-gray-600 mb-3">
            Click on the map to select the {" "}
            <span className="font-medium">{labelForType(pointType)}</span> point.
          </div>

          <div className="relative inline-block select-none">
            <img
              ref={imgRef}
              src={mapConfig.imageUrl}
              alt="Map"
              onLoad={handleImageLoad}
              onClick={handleClick}
              className="max-w-full border rounded-lg cursor-crosshair"
              draggable={false}
            />

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

            {lastClick && (
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

        {/* Controls panel */}
        <div className="bg-white rounded-xl shadow p-4">
          <h3 className="text-lg font-semibold mb-3">Save Point</h3>

          {/* Read-only aisle display */}
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
            {availableTypes.map((t) => (
              <option key={t} value={t}>
                {labelForType(t)}
              </option>
            ))}
          </select>

          <div className="text-xs text-gray-500 mb-4">
            Remaining for {aisleId}:{" "}
            <span className="font-mono">
              {availableTypes.map(labelForType).join(", ") || "none"}
            </span>
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
              <div className="font-mono">{mapConfig.resolution} m/pixel</div>
            </div>

            <div className="rounded-xl border border-gray-200 bg-gray-50 p-3">
              <div className="text-gray-600">Origin</div>
              <div className="font-mono">
                [{mapConfig.origin[0]}, {mapConfig.origin[1]}, {mapConfig.origin[2]}]
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
