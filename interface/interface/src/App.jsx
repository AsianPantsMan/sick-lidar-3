import { Navigate, Route, Routes, useLocation } from "react-router-dom";
import { useEffect, useMemo, useRef, useState } from "react";
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
  const [sortOption, setSortOption] = useState("alphabetical");
  const [showSortMenu, setShowSortMenu] = useState(false);
  const sortMenuRef = useRef(null);

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

  useEffect(() => {
    const handleClickOutside = (event) => {
      if (sortMenuRef.current && !sortMenuRef.current.contains(event.target)) {
        setShowSortMenu(false);
      }
    };

    if (showSortMenu) {
      document.addEventListener("mousedown", handleClickOutside);
      return () => document.removeEventListener("mousedown", handleClickOutside);
    }
  }, [showSortMenu]);

  const handleBackToCategories = () => {
    setCategory("All Products");
    setShowCategoryCards(true);
    setHasInteracted(false);
    setSearchResults(null);
    setQuery("");
    setSortOption("alphabetical");
    setShowSortMenu(false);
  };

  if (!apiBaseUrl) {
    throw new Error("Missing VITE_API_URL.");
  }

  return (
    <div className="app-shell">
      <BodyThemeController pathname={location.pathname} />
      {isCustomerRoute && (
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
            <div className="app-header-sort" ref={sortMenuRef}>
              <button
                type="button"
                className="app-header-sort-button"
                onClick={() => setShowSortMenu((current) => !current)}
                aria-haspopup="menu"
                aria-expanded={showSortMenu}
              >
                Sort by...
              </button>
              {showSortMenu && (
                <div className="app-header-sort-menu" role="menu" aria-label="Sort products">
                  <button
                    type="button"
                    className="app-header-sort-menu-item"
                    onClick={() => {
                      setSortOption("alphabetical");
                      setShowSortMenu(false);
                    }}
                    role="menuitem"
                  >
                    Alphabetical
                  </button>
                  <button
                    type="button"
                    className="app-header-sort-menu-item"
                    onClick={() => {
                      setSortOption("price-high-low");
                      setShowSortMenu(false);
                    }}
                    role="menuitem"
                  >
                    Price High to Low
                  </button>
                  <button
                    type="button"
                    className="app-header-sort-menu-item"
                    onClick={() => {
                      setSortOption("price-low-high");
                      setShowSortMenu(false);
                    }}
                    role="menuitem"
                  >
                    Price Low to High
                  </button>
                </div>
              )}
            </div>
          </div>
        </header>
      )}
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
                sortOption={sortOption}
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
