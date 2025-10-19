export default function CategoryCards({ categories = [], onSelect }) {
  return (
    <div className="grid grid-cols-2 sm:grid-cols-4 gap-4">
      {categories.map((cat) => (
        <button
          key={cat}
          type="button"
          onClick={() => onSelect(cat)}
          className="rounded-lg bg-white shadow-md p-3 flex flex-col items-center justify-center min-h-[220px] min-w-[280px] touch-manipulation focus:outline-none"
          aria-label={`Open ${cat} category`}
        >
          {/* placeholder icon / colored badge */}
          <div
            className="w-12 h-12 rounded-full mb-2"
            style={{
              background:
                "linear-gradient(135deg, #6EE7B7 0%, #3B82F6 50%, #F472B6 100%)",
            }}
            aria-hidden
          />
          <div className="text-4xl font-bold">{cat}</div>
        </button>
      ))}
    </div>
  );
}