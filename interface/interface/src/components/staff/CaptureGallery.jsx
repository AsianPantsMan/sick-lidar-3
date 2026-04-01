import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { getDownloadURL, getMetadata, listAll, ref } from "firebase/storage";
import { storage } from "../../firebase/config";

const IMAGE_EXTENSIONS = [".png", ".jpg", ".jpeg", ".gif", ".webp", ".bmp", ".heic"];
const STORAGE_PATH = import.meta.env.VITE_FIREBASE_STORAGE_PATH || "captures";
const POLL_INTERVAL_MS = 15000;

function isImageFile(name) {
  const lowerName = String(name).toLowerCase();
  return IMAGE_EXTENSIONS.some((extension) => lowerName.endsWith(extension));
}

function formatTimestamp(value) {
  if (!value) return "Unknown time";
  const date = new Date(value);
  return Number.isNaN(date.getTime()) ? "Unknown time" : date.toLocaleString();
}

async function collectStorageImages(folderRef) {
  const snapshot = await listAll(folderRef);
  const nested = await Promise.all(snapshot.prefixes.map((prefixRef) => collectStorageImages(prefixRef)));
  const imageItems = snapshot.items.filter((item) => isImageFile(item.name));
  return [...imageItems, ...nested.flat()];
}

export default function CaptureGallery() {
  const [captures, setCaptures] = useState([]);
  const [notifications, setNotifications] = useState([]);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(null);
  const previousPathsRef = useRef(new Set());
  const hasLoadedOnceRef = useRef(false);

  const storageRootLabel = useMemo(() => `/${STORAGE_PATH}`, []);

  const loadCaptures = useCallback(async () => {
    setLoading(true);
    setError(null);

    try {
      const folderRef = ref(storage, STORAGE_PATH);
      const items = await collectStorageImages(folderRef);
      const capturesWithUrls = await Promise.all(
        items.map(async (itemRef) => {
          const [downloadUrl, metadata] = await Promise.all([getDownloadURL(itemRef), getMetadata(itemRef)]);

          return {
            fullPath: itemRef.fullPath,
            name: itemRef.name,
            url: downloadUrl,
            updatedAt: metadata.updated,
          };
        }),
      );

      capturesWithUrls.sort((a, b) => {
        const aTime = new Date(a.updatedAt || 0).getTime();
        const bTime = new Date(b.updatedAt || 0).getTime();
        if (bTime !== aTime) return bTime - aTime;
        return a.fullPath.localeCompare(b.fullPath);
      });

      const nextPaths = new Set(capturesWithUrls.map((capture) => capture.fullPath));

      if (hasLoadedOnceRef.current) {
        const newlyCaptured = capturesWithUrls.filter((capture) => !previousPathsRef.current.has(capture.fullPath));

        if (newlyCaptured.length > 0) {
          setNotifications((current) => [
            ...newlyCaptured.map((capture) => ({
              id: capture.fullPath,
              title: capture.name,
              url: capture.url,
              createdAt: capture.updatedAt,
            })),
            ...current,
          ].slice(0, 8));
        }
      } else {
        hasLoadedOnceRef.current = true;
      }

      previousPathsRef.current = nextPaths;
      setCaptures(capturesWithUrls);
    } catch (err) {
      console.error("Failed to load storage captures:", err);
      setError(String(err));
    } finally {
      setLoading(false);
    }
  }, []);

  useEffect(() => {
    loadCaptures().catch(console.error);
    const intervalId = window.setInterval(() => {
      loadCaptures().catch(console.error);
    }, POLL_INTERVAL_MS);

    return () => window.clearInterval(intervalId);
  }, [loadCaptures]);

  return (
    <div className="space-y-4">
      <section className="rounded border bg-white p-3 shadow">
        <div className="flex items-center justify-between gap-3 mb-2">
          <div>
            <h3 className="text-lg font-semibold">Capture Notifications</h3>
            <p className="text-xs text-gray-500">Watching {storageRootLabel} for new images.</p>
          </div>
          <button
            type="button"
            onClick={() => setNotifications([])}
            className="text-xs px-2 py-1 rounded bg-zinc-700 text-white hover:bg-zinc-600 transition-colors"
          >
            Clear
          </button>
        </div>

        {notifications.length === 0 ? (
          <div className="rounded border border-dashed border-gray-300 bg-gray-50 p-3 text-sm text-gray-600">
            No new captures yet.
          </div>
        ) : (
          <ul className="space-y-2 max-h-56 overflow-auto">
            {notifications.map((notification) => (
              <li key={notification.id} className="rounded border border-emerald-200 bg-emerald-50 p-3">
                <div className="flex items-start justify-between gap-3">
                  <div>
                    <div className="font-semibold text-emerald-950">New image captured</div>
                    <div className="text-sm text-emerald-900">{notification.title}</div>
                    <div className="text-xs text-emerald-700">{formatTimestamp(notification.createdAt)}</div>
                  </div>
                  <a
                    href={notification.url}
                    target="_blank"
                    rel="noreferrer"
                    className="text-xs font-semibold text-emerald-900 underline"
                  >
                    View
                  </a>
                </div>
              </li>
            ))}
          </ul>
        )}
      </section>

      <section className="rounded border bg-white p-3 shadow">
        <div className="flex items-center justify-between gap-3 mb-2">
          <div>
            <h3 className="text-lg font-semibold">Image Captures</h3>
            <p className="text-xs text-gray-500">Storage path: {storageRootLabel}</p>
          </div>
          <button
            type="button"
            onClick={() => loadCaptures().catch(console.error)}
            className="text-xs px-2 py-1 rounded bg-zinc-700 text-white hover:bg-zinc-600 transition-colors"
          >
            Refresh
          </button>
        </div>

        {loading ? (
          <div className="text-sm text-gray-600">Loading captures...</div>
        ) : error ? (
          <div className="text-sm text-red-600">Error: {error}</div>
        ) : captures.length === 0 ? (
          <div className="rounded border border-dashed border-gray-300 bg-gray-50 p-3 text-sm text-gray-600">
            No image captures found.
          </div>
        ) : (
          <ul className="grid grid-cols-1 gap-3 max-h-[34rem] overflow-auto pr-1">
            {captures.map((capture) => (
              <li key={capture.fullPath} className="overflow-hidden rounded-lg border bg-gray-50 shadow-sm">
                <a href={capture.url} target="_blank" rel="noreferrer" className="block">
                  <div className="aspect-[4/3] w-full bg-black/5">
                    <img src={capture.url} alt={capture.name} className="h-full w-full object-cover" loading="lazy" />
                  </div>
                  <div className="p-3">
                    <div className="font-medium text-slate-900 break-all">{capture.name}</div>
                    <div className="mt-1 text-xs text-gray-600 break-all">{capture.fullPath}</div>
                    <div className="mt-1 text-xs text-gray-500">Updated: {formatTimestamp(capture.updatedAt)}</div>
                  </div>
                </a>
              </li>
            ))}
          </ul>
        )}
      </section>
    </div>
  );
}