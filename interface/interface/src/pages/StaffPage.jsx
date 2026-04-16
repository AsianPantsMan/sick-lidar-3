import { useEffect, useState, useCallback } from "react";
import MapAnnotator from "../MapAnnotator";
import CaptureGallery from "../components/staff/CaptureGallery";
import "../styles/staff.css";

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
              <ul className="text-sm space-y-2 overflow-auto md:max-h-[14rem]">
                {saved.map((p, i) => (
                  <li key={`${p.id}-${p.type}-${p.x}-${p.y}-${i}`} className="border rounded-lg p-2 bg-gray-50">
                    <div className="flex items-start justify-between gap-2">
                      <div>
                        <div className="font-medium">
                          {p.id} - {p.type}
                        </div>
                        <div className="font-mono text-xs">
                          ({Number(p.x).toFixed(3)}, {Number(p.y).toFixed(3)})
                        </div>
                      </div>
                      <button
                        type="button"
                        onClick={() => deleteSavedPoint(i)}
                        className="shrink-0 rounded-full border border-red-300 bg-white px-2 py-0.5 text-xs font-semibold text-red-700 hover:bg-red-50 hover:border-red-400 transition-colors"
                        aria-label={`Delete ${p.id} ${p.type}`}
                        title="Delete point"
                      >
                        x
                      </button>
                    </div>
                  </li>
                ))}
              </ul>
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
