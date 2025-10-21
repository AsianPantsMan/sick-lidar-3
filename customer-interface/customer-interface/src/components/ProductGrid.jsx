export default function ProductGrid({ products }) {
    if (!products.length) return <div>No items found</div>;
    return (
        <div className="grid grid-cols-3 gap-4">
            {products.map(p => (
                <div key={p.productName} className="bg-white rounded shadow p-2 min-h-[220px] min-w-[280px]">
                    <img src={p.image} alt={p.productName} className="w-full h-32 object-cover rounded" />
                    <div className="mt-2 text-m text-gray-600 font-medium">{p.productName}</div>
                    <div className="text-s text-gray-600">{p.productCategory} — ${p.price}</div>
                </div>
            ))}
        </div>
    );
}