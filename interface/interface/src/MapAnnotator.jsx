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

  const availableTypes = useMemo(() => {
    return STEPS.filter((t) => !completedTypes.has(t));
  }, [completedTypes]);

  // Make sure dropdown selection is always valid
  useEffect(() => {
    if (!availableTypes.includes(pointType)) {
      setPointType(availableTypes[0] ?? "start");
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [aisleId, completedTypes]);

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

  async function clearAll() {
    await fetch(`${apiBaseUrl}/api/aisles`, { method: "DELETE" });
    await refreshSaved();

    // reset everything
    setAisleId("A1");
    setPointType("start");
    setCompletedTypes(new Set());
    setLastClick(null);
  }

  if (!mapConfig) {
    return <div className="p-6">Loading map configuration...</div>;
  }

  return (
    <div className="min-h-screen p-6 bg-gray-50">
      <div className="max-w-6xl mx-auto grid grid-cols-1 lg:grid-cols-3 gap-6">
        {/* Map panel */}
        <div className="lg:col-span-2 bg-white rounded-xl shadow p-4">
          <div className="flex items-baseline justify-between mb-3">
              <h2 className="text-xl font-semibold">Aisle Coordinate Picker</h2>
              <div className="text-sm text-gray-700">
              Aisle: <span className="font-mono font-semibold">{aisleId}</span>
            </div>
          </div>

          <div className="text-sm text-gray-600 mb-3">
            Click on the map to select the{" "}
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

            {lastClick && (
              <div
                className="absolute w-3 h-3 rounded-full bg-red-600 border-2 border-white -translate-x-1/2 -translate-y-1/2"
                style={{
                  left: `${lastClick.markerLeft}px`,
                  top: `${lastClick.markerTop}px`,
                }}
                title="Selected point"
              />
            )}
          </div>

          <div className="mt-4 text-sm">
            <div className="grid grid-cols-1 md:grid-cols-2 gap-2">
              <div className="bg-gray-100 rounded p-2">
                <div className="text-gray-600">Resolution</div>
                <div className="font-mono">{mapConfig.resolution} m/pixel</div>
              </div>
              <div className="bg-gray-100 rounded p-2">
                <div className="text-gray-600">Origin</div>
                <div className="font-mono">
                  [{mapConfig.origin[0]}, {mapConfig.origin[1]}, {mapConfig.origin[2]}]
                </div>
              </div>
            </div>

            {lastClick ? (
              <div className="mt-3 bg-green-50 border border-green-200 rounded p-3">
                <div className="font-medium">Selected point</div>
                <div className="font-mono">
                  pixel=({lastClick.px.toFixed(1)}, {lastClick.py.toFixed(1)})
                  <br />
                  world=({lastClick.worldX.toFixed(3)}, {lastClick.worldY.toFixed(3)}) meters
                </div>
              </div>
            ) : (
              <div className="mt-3 text-gray-600">No point selected yet.</div>
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
            disabled={!lastClick}
            className="w-full bg-zinc-800 text-white rounded-md py-2 disabled:opacity-40 hover:bg-zinc-700 transition-colors"
          >
            Save Selected Point
          </button>

          <button
            onClick={clearAll}
            className="w-full mt-2 rounded-md py-2 bg-zinc-600 text-white hover:bg-zinc-500 transition-colors"
          >
            Clear All
          </button>

          <hr className="my-4" />

          <h3 className="text-lg font-semibold mb-2">Saved</h3>
          <div className="max-h-72 overflow-auto border rounded-md">
            {saved.length === 0 ? (
              <div className="p-3 text-sm text-gray-600">No saved points yet.</div>
            ) : (
              <ul className="divide-y">
                {saved.map((p, idx) => (
                  <li key={idx} className="p-3 text-sm">
                    <div className="font-medium text-slate-900">
                      {p.id} — {labelForType(p.type)}
                    </div>
                    <div className="font-mono text-gray-700">
                      ({Number(p.x).toFixed(3)}, {Number(p.y).toFixed(3)})
                    </div>
                    <div className="text-xs text-gray-500">{p.createdAt}</div>
                  </li>
                ))}
              </ul>
            )}
          </div>

        </div>
      </div>
    </div>
  );
}
