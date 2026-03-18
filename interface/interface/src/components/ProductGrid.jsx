import { useState } from "react";
import { X } from "lucide-react";
import storeMap from "../assets/store-map.png";

export default function ProductGrid({ products }) {
    const [selectedProduct, setSelectedProduct] = useState(null);

    if (!products.length) return <div>No items found</div>;

    return (
        <>
            <div className="grid grid-cols-3 gap-4 w-full justify-items-center">
                {products.map(p => (
                    <div 
                        key={p.productName} 
                        className="bg-white rounded-lg shadow-lg h-[280px] w-[280px] relative group overflow-hidden cursor-pointer hover:shadow-2xl transition-shadow"
                        onClick={() => setSelectedProduct(p)}
                    >
                        <img src={p.image} alt={p.productName} className="absolute inset-0 w-full h-full object-cover" />
                        <div className="relative z-10 bg-[#500000] text-white p-3">
                            <div className="text-2xl leading-tight">{p.productName}</div>
                        </div>
                        <div className="absolute bottom-2 right-2 z-20 bg-[#500000] text-white w-24 h-24 rounded-full flex items-center justify-center text-2xl font-mono shadow-md">
                            ${p.price}
                        </div>
                    </div>
                ))}
            </div>

            {selectedProduct && (
                <div 
                    className="fixed inset-0 backdrop-blur-sm flex items-center justify-center z-50"
                    onClick={() => setSelectedProduct(null)}
                >
                    <div 
                        className="bg-[#500000] rounded-2xl shadow-[0_20px_60px_rgba(0,0,0,0.5)] max-w-5xl w-full mx-4 p-8"
                        onClick={(e) => e.stopPropagation()}
                    >
                        <div className="flex justify-between items-center mb-6">
                            <h2 className="text-4xl font-bold text-white">
                                {selectedProduct.productName} can be found on Aisle {selectedProduct.aisleLocation}
                            </h2>
                            <button
                                onClick={() => setSelectedProduct(null)}
                                className="text-white hover:bg-[#600000] rounded-full p-2 transition-colors"
                                aria-label="Close"
                            >
                                <X size={32} />
                            </button>
                        </div>
                        <div className="bg-white rounded-lg p-4">
                            <img 
                                src={storeMap} 
                                alt="Store Map" 
                                className="w-full h-auto"
                            />
                        </div>
                    </div>
                </div>
            )}
        </>
    );
}