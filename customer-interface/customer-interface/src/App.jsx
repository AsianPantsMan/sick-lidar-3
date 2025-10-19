import { useState } from "react";
import "./App.css";
import SearchBar from "./components/SearchBar";
import CategoryCards from "./components/CategoryCards";
import ProductGrid from "./components/ProductGrid";
import useProducts from "./hooks/useProducts";

function App() {
  const [query, setQuery] = useState("");
  const [category, setCategory] = useState("All");
  const products = useProducts(); // returns full list (mock/fetch)

  // filter locally for snappy touch UX
  const visible = products.filter((p) => {
    const matchesCategory = category === "All" || p.productCategory === category;
    const matchesQuery = query.trim() === "" || p.productName.toLowerCase().includes(query.toLowerCase());
    return matchesCategory && matchesQuery;
  });

  const categories = ["All", "Produce", "Dairy", "Bakery", "Meat", "Frozen", "Pantry", "Household"];

  return (
    <div className="p-4">
      <SearchBar value={query} onChange={setQuery} placeholder="Search items..." />

      {category === "All" ? (
        // show category cards in the grid by default
        <CategoryCards
          categories={categories.filter((c) => c !== "All")}
          onSelect={(c) => setCategory(c)}
        />
      ) : (
        // show selected category's products; provide a simple way back to categories
        <>
          <div className="flex items-center gap-2 mb-3">
            <button
              onClick={() => setCategory("All")}
              className="px-3 py-1 rounded bg-gray-100 text-sm"
              aria-label="Back to categories"
            >
              ← Categories
            </button>
            <h2 className="text-lg font-semibold">{category}</h2>
          </div>
          <ProductGrid products={visible} />
        </>
      )}
    </div>
  );
}

export default App;
