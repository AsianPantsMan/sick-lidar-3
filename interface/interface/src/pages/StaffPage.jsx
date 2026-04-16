import { useEffect, useState, useCallback, useMemo } from "react";
import MapAnnotator from "../MapAnnotator";
import CaptureGallery from "../components/staff/CaptureGallery";
import "../styles/staff.css";

function labelForType(type) {
  if (type === "start") return "begin";
  if (type === "center") return "center";
  return "end";
}

function typeOrder(type) {
  if (type === "start") return 0;
  if (type === "center") return 1;
  return 2;
}

function aisleSortKey(aisleId) {
  const match = String(aisleId).trim().match(/^([A-Za-z]+)(\d+)$/);
  if (!match) return { prefix: String(aisleId), number: Number.POSITIVE_INFINITY };
  return { prefix: match[1].toUpperCase(), number: Number.parseInt(match[2], 10) };
}

function compareAisleIds(left, right) {
  const a = aisleSortKey(left);
  const b = aisleSortKey(right);
  if (a.prefix !== b.prefix) return a.prefix.localeCompare(b.prefix);
  if (a.number !== b.number) return a.number - b.number;
  return String(left).localeCompare(String(right));
}

export default function StaffPage() {
  const [saved, setSaved] = useState([]);
  const [loadingSaved, setLoadingSaved] = useState(false);
  const [savedError, setSavedError] = useState(null);
  const apiBaseUrl = import.meta.env.VITE_API_URL;

  if (!apiBaseUrl) {
    throw new Error("Missing VITE_API_URL.");
  }

  const refreshSaved = useCallback(async () => {
    setLoadingSaved(true);
    setSavedError(null);
    try {
      const resp = await fetch(`${apiBaseUrl}/api/aisles`, { cache: "no-store" });
      if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
      const data = await resp.json();
      const nextSaved = data.aisles || [];
      setSaved(nextSaved);
      return nextSaved;
    } catch (err) {
      console.error("Failed to load saved aisles:", err);
      setSavedError(String(err));
      return [];
    } finally {
      setLoadingSaved(false);
    }
  }, [apiBaseUrl]);

  const clearAllSaved = useCallback(async () => {
    await fetch(`${apiBaseUrl}/api/aisles`, { method: "DELETE", cache: "no-store" });
    await refreshSaved();
  }, [apiBaseUrl, refreshSaved]);

  const deleteSavedPoint = useCallback(async (index) => {
    await fetch(`${apiBaseUrl}/api/aisles/${index}`, {
      method: "DELETE",
      cache: "no-store",
    });
    await refreshSaved();
  }, [apiBaseUrl, refreshSaved]);

  const groupedSavedPoints = useMemo(() => {
    const ordered = saved
      .map((point, index) => ({ point, index }))
      .sort((left, right) => {
        const aisleComparison = compareAisleIds(left.point.id, right.point.id);
        if (aisleComparison !== 0) return aisleComparison;

        const typeComparison = typeOrder(left.point.type) - typeOrder(right.point.type);
        if (typeComparison !== 0) return typeComparison;

        return left.index - right.index;
      });

    return ordered.reduce((groups, entry) => {
      const aisleId = String(entry.point.id);
      const group = groups.get(aisleId) || [];
      group.push(entry);
      groups.set(aisleId, group);
      return groups;
    }, new Map());
  }, [saved]);

  useEffect(() => {
    refreshSaved();
    const intervalId = window.setInterval(() => {
      refreshSaved().catch(console.error);
    }, 15000);

    return () => window.clearInterval(intervalId);
  }, [refreshSaved]);

  return (
    <div className="App p-6 max-w-6xl mx-auto">

      <div className="mb-4 grid grid-cols-1 md:grid-cols-3 gap-4 items-stretch">
        <div className="col-span-1 md:col-span-2 h-full">
          <MapAnnotator saved={saved} refreshSaved={refreshSaved} apiBaseUrl={apiBaseUrl} />
        </div>

        <aside className="col-span-1 space-y-4 md:h-full md:min-h-0 md:flex md:flex-col">
          <div className="bg-white border rounded-xl p-3 shadow md:flex-none md:max-h-[20rem] md:flex md:flex-col">
            <div className="flex items-center justify-between mb-2 gap-2">
              <h3 className="font-semibold text-lg">Saved Points</h3>
              <div className="flex items-center gap-2">
                <button
                  onClick={clearAllSaved}
                  className="text-xs px-2 py-1 rounded-xl bg-zinc-600 text-white hover:bg-zinc-500 transition-colors"
                >
                  Clear All
                </button>
                <button
                  onClick={refreshSaved}
                  className="text-xs px-2 py-1 rounded-xl bg-red-900 text-white hover:bg-red-950 transition-colors"
                >
                  Refresh
                </button>
              </div>
            </div>

            {loadingSaved ? (
              <div className="text-sm text-gray-600">Loading...</div>
            ) : savedError ? (
              <div className="text-sm text-red-600">Error: {savedError}</div>
            ) : saved.length === 0 ? (
              <div className="text-sm text-gray-600">No saved points.</div>
            ) : (
              <div className="space-y-3 overflow-auto md:max-h-[14rem] pr-1">
                {Array.from(groupedSavedPoints.entries()).map(([aisleId, entries]) => (
                  <section key={aisleId} className="rounded-lg border border-gray-200 bg-gray-50 p-1.5">
                    <div className="mb-1.5 flex items-center justify-between gap-2">
                      <h4 className="text-sm font-semibold text-gray-800">Aisle {aisleId}</h4>
                    
                    </div>
                    <ul className="space-y-1.5">
                      {entries.map(({ point, index }) => (
                        <li key={`${point.id}-${point.type}-${point.x}-${point.y}-${index}`} className="rounded-md border bg-white p-1.5">
                          <div className="flex items-center justify-between gap-2">
                            <div className="min-w-0 flex items-center gap-2 text-sm leading-tight">
                              <span className="font-medium text-gray-900 whitespace-nowrap">
                                {labelForType(point.type)}
                              </span>
                              <span className="font-mono text-[11px] text-gray-700 whitespace-nowrap">
                                ({Number(point.x).toFixed(3)}, {Number(point.y).toFixed(3)})
                              </span>
                            </div>
                            <button
                              type="button"
                              onClick={() => deleteSavedPoint(index)}
                              className="shrink-0 text-xs px-2 py-1 rounded-xl bg-red-900 text-white hover:bg-red-950 transition-colors"
                              aria-label={`Delete ${point.id} ${point.type}`}
                              title="Delete point"
                            >
                              x
                            </button>
                          </div>
                        </li>
                      ))}
                    </ul>
                  </section>
                ))}
              </div>
            )}
          </div>

          <div className="md:flex-[2] md:min-h-0">
            <CaptureGallery />
          </div>
        </aside>
      </div>
    </div>
  );
}
