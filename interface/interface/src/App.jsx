import { Navigate, Route, Routes, useLocation } from "react-router-dom";
import { useEffect } from "react";
import "./App.css";
import CustomerPage from "./pages/CustomerPage";
import StaffPage from "./pages/StaffPage";

function BodyThemeController() {
  const location = useLocation();

  useEffect(() => {
    const isCustomerRoute = location.pathname.startsWith("/customer");
    const isStaffRoute = location.pathname.startsWith("/staff");

    document.body.classList.toggle("customer-theme", isCustomerRoute);
    document.body.classList.toggle("staff-theme", isStaffRoute);

    return () => {
      document.body.classList.remove("customer-theme");
      document.body.classList.remove("staff-theme");
    };
  }, [location.pathname]);

  return null;
}

export default function App() {
  return (
    <div className="app-shell">
      <BodyThemeController />
      <main className="app-main">
        <Routes>
          <Route path="/" element={<Navigate to="/customer" replace />} />
          <Route path="/customer" element={<CustomerPage />} />
          <Route path="/staff" element={<StaffPage />} />
        </Routes>
      </main>
    </div>
  );
}
