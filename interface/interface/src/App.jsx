import { Navigate, Route, Routes, useLocation } from "react-router-dom";
import { useEffect, useMemo, useState } from "react";
import "./App.css";
import CustomerPage from "./pages/CustomerPage";
import StaffPage from "./pages/StaffPage";
import revIcon from "./assets/rev.ico";
import { ArrowLeft } from "lucide-react";
import SearchBar from "./components/SearchBar";
import useProducts from "./hooks/useProducts";
import useSelectedMap from "./hooks/useSelectedMap";

function BodyThemeController({ pathname }) {

  useEffect(() => {
    const isCustomerRoute = pathname.startsWith("/customer");
    const isStaffRoute = pathname.startsWith("/staff");

    document.body.classList.toggle("customer-theme", isCustomerRoute);
    document.body.classList.toggle("staff-theme", isStaffRoute);

    return () => {
      document.body.classList.remove("customer-theme");
      document.body.classList.remove("staff-theme");
    };
  }, [pathname]);

  return null;
}

export default function App() {
  const location = useLocation();
  const apiBaseUrl = import.meta.env.VITE_API_URL;
  const products = useProducts();
  const activeMap = useSelectedMap(apiBaseUrl);

  const [query, setQuery] = useState("");
  const [category, setCategory] = useState("All Products");
  const [showCategoryCards, setShowCategoryCards] = useState(true);
  const [hasInteracted, setHasInteracted] = useState(false);
  const [searchResults, setSearchResults] = useState(null);

  const isCustomerRoute = location.pathname.startsWith("/customer");

  const customerHeaderTitle = useMemo(() => {
    if (!hasInteracted && category === "All Products" && query.trim() === "") {
      return "Select a Product Category";
    }

    if (category === "All Products" && query.trim() === "") {
      return "All Products";
    }

    if (category === "All Products" && query.trim() !== "") {
      return "Search Results";
    }

    return category;
  }, [category, hasInteracted, query]);

  const customerSearchValue = query;
  const headerTitle = isCustomerRoute ? customerHeaderTitle : "Retail Assistant";
  const showHeaderBackButton = isCustomerRoute && !showCategoryCards;

  const handleBackToCategories = () => {
    setCategory("All Products");
    setShowCategoryCards(true);
    setHasInteracted(false);
    setSearchResults(null);
    setQuery("");
  };

  if (!apiBaseUrl) {
    throw new Error("Missing VITE_API_URL.");
  }

  return (
    <div className="app-shell">
      <BodyThemeController pathname={location.pathname} />
      <header className="app-header" aria-label="Retail Assistant navigation bar">
        <div className="app-header-brand">
          {showHeaderBackButton && (
            <button
              type="button"
              onClick={handleBackToCategories}
              className="app-header-back-button"
              aria-label="Back to categories"
            >
              <ArrowLeft size={22} />
              <span>Back</span>
            </button>
          )}
          <div className="app-header-brand-content">
            <img src={revIcon} alt="Retail Assistant logo" className="app-header-logo" />
            <span className="app-header-title">{headerTitle}</span>
          </div>
        </div>
        <div className="app-header-fill">
          {isCustomerRoute && (
            <>
              <div className="app-header-search">
                <SearchBar
                  value={customerSearchValue}
                  onChange={setQuery}
                  placeholder="Search"
                  category={category}
                  products={products}
                  onResults={setSearchResults}
                />
              </div>
            </>
          )}
        </div>
      </header>
      <main className="app-main">
        <Routes>
          <Route path="/" element={<Navigate to="/customer" replace />} />
          <Route
            path="/customer"
            element={
              <CustomerPage
                category={category}
                setCategory={setCategory}
                showCategoryCards={showCategoryCards}
                setShowCategoryCards={setShowCategoryCards}
                hasInteracted={hasInteracted}
                setHasInteracted={setHasInteracted}
                query={query}
                setQuery={setQuery}
                searchResults={searchResults}
                setSearchResults={setSearchResults}
                products={products}
                activeMap={activeMap.mapConfig}
                activeMapLoading={activeMap.mapLoading}
              />
            }
          />
          <Route path="/staff" element={<StaffPage />} />
        </Routes>
      </main>
    </div>
  );
}
