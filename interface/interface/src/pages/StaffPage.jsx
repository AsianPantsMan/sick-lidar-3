import { useEffect, useState, useCallback, useMemo, useRef } from "react";
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

const POINT_TYPES = ["start", "center", "end"];

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
  const savedListRef = useRef(null);
  const restoreScrollTopRef = useRef(null);
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
      if (typeof restoreScrollTopRef.current === "number") {
        const nextScrollTop = restoreScrollTopRef.current;
        window.requestAnimationFrame(() => {
          if (savedListRef.current) {
            savedListRef.current.scrollTop = nextScrollTop;
          }
          restoreScrollTopRef.current = null;
        });
      }
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
    restoreScrollTopRef.current = savedListRef.current?.scrollTop ?? 0;
    await fetch(`${apiBaseUrl}/api/aisles/${index}`, {
      method: "DELETE",
      cache: "no-store",
    });
    await refreshSaved();
  }, [apiBaseUrl, refreshSaved]);

  const deleteAislePoints = useCallback(async (entries) => {
    if (!entries || entries.length === 0) return;

    restoreScrollTopRef.current = savedListRef.current?.scrollTop ?? 0;

    const indices = entries
      .map((entry) => entry.index)
      .sort((a, b) => b - a);

    for (const index of indices) {
      await fetch(`${apiBaseUrl}/api/aisles/${index}`, {
        method: "DELETE",
        cache: "no-store",
      });
    }

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
    return undefined;
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

            {loadingSaved && saved.length === 0 ? (
              <div className="text-sm text-gray-600">Loading...</div>
            ) : savedError ? (
              <div className="text-sm text-red-600">Error: {savedError}</div>
            ) : saved.length === 0 ? (
              <div className="text-sm text-gray-600">No saved points.</div>
            ) : (
              <div ref={savedListRef} className="space-y-3 overflow-auto md:max-h-[14rem] pr-1">
                {Array.from(groupedSavedPoints.entries()).map(([aisleId, entries]) => (
                  <section key={aisleId} className="rounded-lg border border-gray-200 bg-gray-50 p-1.5">
                    <div className="mb-1.5 flex items-center justify-between gap-2">
                      <h4 className="text-sm font-semibold text-gray-800">Aisle {aisleId}</h4>
                      <button
                        type="button"
                        onClick={() => deleteAislePoints(entries)}
                        className="text-[10px] px-1.5 py-0.5 rounded-md bg-zinc-600 text-white hover:bg-zinc-500 transition-colors"
                        aria-label={`Delete all points for aisle ${aisleId}`}
                        title={`Delete all points for aisle ${aisleId}`}
                      >
                        X
                      </button>
                    </div>
                    <ul className="grid grid-cols-3 gap-1">
                      {POINT_TYPES.map((type) => {
                        const entry = entries.find(({ point }) => point.type === type);

                        if (!entry) {
                          return (
                            <li key={`${aisleId}-${type}`}>
                              <div className="w-full rounded-md border border-dashed bg-white/70 p-1 text-left">
                                <div className="text-[11px] font-semibold text-gray-500 leading-tight">{labelForType(type)}</div>
                                <div className="font-mono text-[10px] text-gray-400 leading-tight mt-0.5">( - , - )</div>
                              </div>
                            </li>
                          );
                        }

                        const { point, index } = entry;

                        return (
                          <li key={`${point.id}-${point.type}-${point.x}-${point.y}-${index}`}>
                            <button
                              type="button"
                              onClick={() => deleteSavedPoint(index)}
                              className="w-full rounded-md border bg-white p-1 text-left hover:bg-red-50 hover:border-red-200 transition-colors"
                              aria-label={`Delete ${point.id} ${point.type}`}
                              title={`Delete ${point.id} ${point.type}`}
                            >
                              <div className="text-[11px] font-semibold text-gray-900 leading-tight">
                                {labelForType(point.type)}
                              </div>
                              <div className="font-mono text-[10px] text-gray-700 leading-tight mt-0.5">
                                ({Number(point.x).toFixed(2)}, {Number(point.y).toFixed(2)})
                              </div>
                            </button>
                          </li>
                        );
                      })}
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
