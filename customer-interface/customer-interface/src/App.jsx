import { useState } from "react";
import "./App.css";
import SearchBar from "./components/SearchBar";
import CategoryCards from "./components/CategoryCards";
import ProductGrid from "./components/ProductGrid";
import useProducts from "./hooks/useProducts";

function App() {
  const [query, setQuery] = useState("");
  const [category, setCategory] = useState("All");
  const [searchResults, setSearchResults] = useState(null); // <-- receive debounced results from SearchBar
  const products = useProducts(); // returns full list (mock/fetch)

  // visible used for category-specific views and when no query
  const visible = products.filter((p) => {
    const matchesCategory = category === "All" || p.productCategory === category;
    const matchesQuery = query.trim() === "" || p.productName.toLowerCase().includes(query.toLowerCase());
    return matchesCategory && matchesQuery;
  });

  const categories = ["All", "Produce", "Dairy", "Bakery", "Meat", "Frozen", "Pantry", "Household"];

  // When on the category grid (category === "All") and a query exists, show searchResults (or fallback to filtered visible)
  const isSearchingOnCategoryGrid = category === "All" && query.trim() !== "";
  const gridProducts = isSearchingOnCategoryGrid ? (searchResults ?? []) : visible;

  return (
    <div className="p-4">
      <SearchBar
        value={query}
        onChange={setQuery}
        placeholder="Search items..."
        category={category}
        products={products}
        onResults={setSearchResults}
      />

      {category === "All" && query.trim() === "" ? (
        // show category cards in the grid by default (no query)
        <CategoryCards
          categories={categories.filter((c) => c !== "All")}
          onSelect={(c) => setCategory(c)}
        />
      ) : (
        // show product grid either for selected category or when searching on category grid
        <>
          {category !== "All" && (
            <div className="flex items-center gap-2 mb-3">
              <button
                onClick={() => {
                  setCategory("All");
                }}
                className="px-3 py-1 rounded bg-gray-100 text-sm"
                aria-label="Back to categories"
              >
                ← Categories
              </button>
              <h2 className="text-lg font-semibold">{category}</h2>
            </div>
          )}
          <ProductGrid products={gridProducts} />
        </>
      )}
    </div>
  );
}

export default App;
