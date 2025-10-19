export default function ProductGrid({ products }) {
    if (!products.length) return <div>No items found</div>;
    return (
        <div className="grid grid-cols-2 gap-3">
            {products.map(p => (
                <div key={p.productName} className="bg-white rounded shadow p-2">
                    <img src={p.image} alt={p.productName} className="w-full h-32 object-cover rounded" />
                    <div className="mt-2 text-sm font-medium">{p.productName}</div>
                    <div className="text-xs text-gray-600">{p.productCategory} — ${p.price}</div>
                </div>
            ))}
        </div>
    );
}