import { useEffect, useState } from "react";
import { ArrowLeft } from "lucide-react";
import SearchBar from "../components/SearchBar";
import CategoryCards from "../components/CategoryCards";
import ProductGrid from "../components/ProductGrid";
import useProducts from "../hooks/useProducts";
import "../styles/customer.css";

export default function CustomerPage() {
  const [query, setQuery] = useState("");
  const [category, setCategory] = useState("All Products");
  const [showCategoryCards, setShowCategoryCards] = useState(true);
  const [hasInteracted, setHasInteracted] = useState(false);
  const [searchResults, setSearchResults] = useState(null);
  const products = useProducts();

  useEffect(() => {
    if (query.trim() !== "") {
      setShowCategoryCards(false);
      setHasInteracted(true);
    }
  }, [query]);

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
    <div className="customer-shell min-h-screen">
      <div className="text-5xl font-bold p-10 text-center">
        {!hasInteracted && category === "All Products" && query.trim() === ""
          ? "Select a product category or search"
          : category === "All Products" && query.trim() === ""
            ? "All Products"
            : category === "All Products" && query.trim() !== ""
              ? "Search Results"
              : category}
      </div>
      <div className="p-4">
        <div className="flex items-center justify-center gap-4 mb-10 max-w-5xl mx-auto w-full">
          {!showCategoryCards && (
            <button
              onClick={() => {
                setCategory("All Products");
                setShowCategoryCards(true);
                setHasInteracted(false);
                setQuery("");
              }}
              className="px-6 py-4 rounded-xl bg-[#500000] text-white text-lg font-bold shadow-lg hover:bg-[#600000] transition-colors whitespace-nowrap flex items-center gap-2 h-[62px]"
              aria-label="Back to categories"
            >
              <ArrowLeft size={24} />
              Back
            </button>
          )}
          <div className="flex-1 w-full">
            <SearchBar
              value={query}
              onChange={setQuery}
              placeholder="Search"
              category={category}
              products={products}
              onResults={setSearchResults}
            />
          </div>
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
          <ProductGrid products={gridProducts} />
        )}
      </div>
    </div>
  );
}
