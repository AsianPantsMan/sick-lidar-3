import { useState } from "react";
import { X } from "lucide-react";

export default function ProductGrid({ products, activeMap, activeMapLoading }) {
    const [selectedProduct, setSelectedProduct] = useState(null);

    if (!products.length) return <div>No items found</div>;

    return (
        <>
            <div className="grid grid-cols-3 gap-2 w-full justify-items-center justify-center">
                {products.map(p => (
                    <div 
                        key={p.productName} 
                        className="bg-white rounded-lg shadow-xl h-[250px] w-[380px] relative group overflow-hidden cursor-pointer hover:shadow-2xl transition-shadow flex flex-col"
                        onClick={() => setSelectedProduct(p)}
                    >
                        <div className="relative z-10 bg-[#500000] text-white p-3 flex items-start justify-between gap-3">
                            <div className="min-w-0">
                                <div className="text-[0.7rem] uppercase tracking-[0.16em] text-white/75 leading-none mb-1">
                                    {p.productCategory}
                                </div>
                                <div className="text-2xl font-bold leading-tight">
                                    {p.productName}
                                </div>
                            </div>
                            <div className="shrink-0 bg-white text-[#500000] px-3 py-1 rounded-full text-lg font-bold shadow-sm">
                                ${p.price}
                            </div>
                        </div>
                        <div className="flex-1 p-3 bg-white">
                            <img src={p.image} alt={p.productName} className="w-full h-full object-cover rounded-md" />
                        </div>
                    </div>
                ))}
            </div>

            {selectedProduct && (
                <div 
                    className="fixed inset-0 backdrop-blur-sm flex items-center justify-center z-50 p-4"
                    onClick={() => setSelectedProduct(null)}
                >
                    <div 
                        className="bg-[#500000] rounded-2xl shadow-[0_20px_60px_rgba(0,0,0,0.5)] w-[min(99vw,1680px)] h-[96vh] flex flex-col overflow-hidden p-5 md:p-8"
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
                        <div className="bg-white rounded-lg p-2 md:p-3 flex-1 min-h-0 overflow-hidden flex items-center justify-center">
                            {activeMapLoading && !activeMap ? (
                                <div className="w-full h-full flex items-center justify-center text-white bg-[#500000] rounded-lg">
                                    Loading selected map...
                                </div>
                            ) : activeMap ? (
                                <img 
                                    src={activeMap.imageUrl} 
                                    alt={activeMap.sourcePath ? `Selected map ${activeMap.sourcePath}` : "Selected map"} 
                                    className="block w-full h-full object-contain"
                                />
                            ) : (
                                <div className="w-full h-full flex items-center justify-center text-gray-700 bg-gray-100 rounded-lg">
                                    No selected map available.
                                </div>
                            )}
                        </div>
                    </div>
                </div>
            )}
        </>
    );
}