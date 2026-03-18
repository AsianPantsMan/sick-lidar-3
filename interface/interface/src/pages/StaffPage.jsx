import { useEffect, useState, useCallback } from "react";
import MapAnnotator from "../MapAnnotator";
import "../styles/staff.css";

export default function StaffPage() {
  const [isLoggedIn, setIsLoggedIn] = useState(false);
  const [saved, setSaved] = useState([]);
  const [loadingSaved, setLoadingSaved] = useState(false);
  const [savedError, setSavedError] = useState(null);

  const refreshSaved = useCallback(async () => {
    setLoadingSaved(true);
    setSavedError(null);
    try {
      const resp = await fetch("http://localhost:5050/api/aisles");
      if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
      const data = await resp.json();
      setSaved(data.aisles || []);
    } catch (err) {
      console.error("Failed to load saved aisles:", err);
      setSavedError(String(err));
    } finally {
      setLoadingSaved(false);
    }
  }, []);

  useEffect(() => {
    if (isLoggedIn) refreshSaved();
  }, [isLoggedIn, refreshSaved]);

  return (
    <div className="App p-6 max-w-6xl mx-auto">
      <h1 className="text-2xl font-bold mb-4">Staff Portal</h1>

      {!isLoggedIn ? (
        <Login onSuccess={() => setIsLoggedIn(true)} />
      ) : (
        <div className="mb-4 grid grid-cols-1 md:grid-cols-3 gap-4">
          <div className="col-span-1 md:col-span-2">
            <h2 className="text-lg font-semibold">Map Annotator</h2>
            <MapAnnotator saved={saved} refreshSaved={refreshSaved} />
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
      )}
    </div>
  );
}

function Login({ onSuccess }) {
  const [username, setUsername] = useState("");
  const [password, setPassword] = useState("");
  const [err, setErr] = useState(null);

  function handleSubmit(e) {
    e.preventDefault();
    setErr(null);

    if (username.trim() === "" || password.trim() === "") {
      setErr("Please enter both username and password.");
      return;
    }

    onSuccess();
  }

  return (
    <form onSubmit={handleSubmit} className="p-6 space-y-4 max-w-md mx-auto bg-white border rounded shadow">
      <label className="block text-sm">
        <span className="text-gray-700">Username</span>
        <input
          className="mt-1 block w-full border rounded-md px-3 py-2"
          type="text"
          value={username}
          onChange={(e) => setUsername(e.target.value)}
          placeholder="username"
          autoComplete="username"
        />
      </label>

      <label className="block text-sm">
        <span className="text-gray-700">Password</span>
        <input
          className="mt-1 block w-full border rounded-md px-3 py-2"
          type="password"
          value={password}
          onChange={(e) => setPassword(e.target.value)}
          placeholder="password"
          autoComplete="current-password"
        />
      </label>

      {err && <div className="text-sm text-red-600">{err}</div>}

      <button type="submit" className="w-full bg-black text-white rounded-md py-2">
        Login
      </button>
    </form>
  );
}
