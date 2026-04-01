import { useEffect, useState, useCallback } from "react";
import MapAnnotator from "../MapAnnotator";
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
      const resp = await fetch(`${apiBaseUrl}/api/aisles`);
      if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
      const data = await resp.json();
      setSaved(data.aisles || []);
    } catch (err) {
      console.error("Failed to load saved aisles:", err);
      setSavedError(String(err));
    } finally {
      setLoadingSaved(false);
    }
  }, [apiBaseUrl]);

  useEffect(() => {
    refreshSaved();
  }, [refreshSaved]);

  return (
    <div className="App p-6 max-w-6xl mx-auto">
      <h1 className="text-2xl font-bold mb-4">Staff Portal</h1>

      <div className="mb-4 rounded border border-amber-300 bg-amber-50 px-4 py-3 text-sm text-amber-900">
        Login is temporarily disabled while the hosted backend is in use.
      </div>

      <div className="mb-4 grid grid-cols-1 md:grid-cols-3 gap-4">
        <div className="col-span-1 md:col-span-2">
          <h2 className="text-lg font-semibold">Map Annotator</h2>
          <MapAnnotator saved={saved} refreshSaved={refreshSaved} apiBaseUrl={apiBaseUrl} />
        </div>

        <aside className="col-span-1 bg-white border rounded p-3 shadow">
          <div className="flex items-center justify-between mb-2">
            <h3 className="font-semibold">Saved Points</h3>
            <button onClick={refreshSaved} className="text-xs px-2 py-1 border rounded">
              Refresh
            </button>
          </div>

          {loadingSaved ? (
            <div className="text-sm text-gray-600">Loading...</div>
          ) : savedError ? (
            <div className="text-sm text-red-600">Error: {savedError}</div>
          ) : saved.length === 0 ? (
            <div className="text-sm text-gray-600">No saved points.</div>
          ) : (
            <ul className="text-sm space-y-2 max-h-[60vh] overflow-auto">
              {saved.map((p, i) => (
                <li key={i} className="border rounded p-2 bg-gray-50">
                  <div className="font-medium">
                    {p.id} - {p.type}
                  </div>
                  <div className="font-mono text-xs">
                    ({Number(p.x).toFixed(3)}, {Number(p.y).toFixed(3)})
                  </div>
                  {p.createdAt && <div className="text-xs text-gray-500">{p.createdAt}</div>}
                </li>
              ))}
            </ul>
          )}
        </aside>
      </div>
    </div>
  );
}
