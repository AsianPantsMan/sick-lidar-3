export default function CategoryList({ categories, selected, onSelect }) {
    return (
        <div className="flex gap-2 overflow-x-auto py-2 mb-4">
            {categories.map(cat => (
                <button
                    key={cat}
                    onClick={() => onSelect(cat)}
                    className={`min-w-[64px] px-4 py-2 rounded-full text-sm ${
                        selected === cat ? "bg-blue-600 text-white" : "bg-gray-100"
                    }`}
                    aria-pressed={selected === cat}
                >
                    {cat}
                </button>
            ))}
        </div>
    );
}