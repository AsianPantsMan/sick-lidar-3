import { useEffect } from "react";
import { ArrowLeft } from "lucide-react";
import CategoryCards from "../components/CategoryCards";
import ProductGrid from "../components/ProductGrid";
import "../styles/customer.css";

export default function CustomerPage({
  category,
  setCategory,
  showCategoryCards,
  setShowCategoryCards,
  hasInteracted,
  setHasInteracted,
  query,
  searchResults,
  setSearchResults,
  products,
  activeMap,
  activeMapLoading,
}) {
  useEffect(() => {
    if (query.trim() !== "") {
      setShowCategoryCards(false);
      setHasInteracted(true);
    }
  }, [query, setHasInteracted, setShowCategoryCards]);

  const visible = products.filter((p) => {
    const matchesCategory = category === "All Products" || p.productCategory === category;
    const matchesQuery = query.trim() === "" || p.productName.toLowerCase().includes(query.toLowerCase());
    return matchesCategory && matchesQuery;
  });

  const sortedVisible =
    category === "All Products" ? [...visible].sort((a, b) => a.productName.localeCompare(b.productName)) : visible;

  const categories = [
    "All Products",
    "Dry Goods & Grains",
    "Baking & Canned",
    "Snacks & Breakfast",
    "Beverages",
    "Office Supplies",
    "Pet Supplies",
    "Household & Personal Care",
  ];

  const isSearchingOnCategoryGrid = category === "All Products" && query.trim() !== "";
  const gridProducts = isSearchingOnCategoryGrid ? (searchResults ?? []) : sortedVisible;

  return (
    <div className="customer-shell min-h-screen p-4">
      <div className="flex items-center justify-start gap-4 mb-8 max-w-7xl mx-auto w-full">
        {!showCategoryCards && (
          <button
            onClick={() => {
              setCategory("All Products");
              setShowCategoryCards(true);
              setHasInteracted(false);
              setSearchResults(null);
              setQuery("");
            }}
            className="px-6 py-4 rounded-xl bg-[#500000] text-white text-lg font-bold shadow-lg hover:bg-[#600000] transition-colors whitespace-nowrap flex items-center gap-2 h-[62px]"
            aria-label="Back to categories"
          >
            <ArrowLeft size={24} />
            Back
          </button>
        )}
      </div>

      {showCategoryCards && query.trim() === "" ? (
        <CategoryCards
          categories={categories}
          onSelect={(c) => {
            setCategory(c);
            setShowCategoryCards(false);
            setHasInteracted(true);
          }}
        />
      ) : (
        <ProductGrid products={gridProducts} activeMap={activeMap} activeMapLoading={activeMapLoading} />
      )}
    </div>
  );
}
