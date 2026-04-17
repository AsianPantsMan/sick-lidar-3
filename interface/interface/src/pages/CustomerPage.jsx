import { useEffect } from "react";
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
  products,
  sortOption,
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

  const sortProducts = (items) => {
    const getPriceValue = (product) => Number(String(product.price ?? 0).replace(/[^0-9.-]/g, "")) || 0;

    const sortedItems = [...items];

    if (sortOption === "price-high-low") {
      sortedItems.sort((a, b) => getPriceValue(b) - getPriceValue(a));
      return sortedItems;
    }

    if (sortOption === "price-low-high") {
      sortedItems.sort((a, b) => getPriceValue(a) - getPriceValue(b));
      return sortedItems;
    }

    sortedItems.sort((a, b) => a.productName.localeCompare(b.productName));
    return sortedItems;
  };

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
  const gridProducts = sortProducts(isSearchingOnCategoryGrid ? (searchResults ?? []) : sortedVisible);

  return (
    <div className="customer-shell p-4">
      {showCategoryCards && query.trim() === "" ? (
        <div className="customer-categories-stage">
          <CategoryCards
            categories={categories}
            onSelect={(c) => {
              setCategory(c);
              setShowCategoryCards(false);
              setHasInteracted(true);
            }}
          />
        </div>
      ) : (
        <ProductGrid products={gridProducts} activeMap={activeMap} activeMapLoading={activeMapLoading} />
      )}
    </div>
  );
}
