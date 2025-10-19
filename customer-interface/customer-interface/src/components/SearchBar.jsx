import { useState, useEffect } from "react";

/**
 * Props:
 * - value, onChange, placeholder (existing)
 * - category (string) - current selected category, e.g. "All" or "Produce"
 * - products (array) - optional list of product objects to perform client-side search
 * - onResults (function) - optional callback receiving filtered results
 */
export default function SearchBar({
  value,
  onChange,
  placeholder,
  category = "All",
  products = [],
  onResults = () => {},
}) {
  const [local, setLocal] = useState(value);
  useEffect(() => setLocal(value), [value]);

  // debounce search + optionally filter products and emit results
  useEffect(() => {
    const t = setTimeout(() => {
      onChange(local); // keep existing contract (query string)

      if (Array.isArray(products) && products.length > 0) {
        const q = local.trim().toLowerCase();
        const filtered = products.filter((p) => {
          const inCategory = category === "All" || p.productCategory === category;
          const matchesQuery = q === "" || (p.productName || "").toLowerCase().includes(q);
          return inCategory && matchesQuery;
        });
        onResults(filtered);
      }
    }, 250);

    return () => clearTimeout(t);
  }, [local, onChange, products, onResults, category]);

  return (
    <div className="mb-3">
      <input
        type="search"
        inputMode="search"
        value={local}
        onChange={(e) => setLocal(e.target.value)}
        placeholder={placeholder}
        className="w-full p-3 rounded-lg border focus:outline-none"
        aria-label="Search products"
      />
    </div>
  );
}