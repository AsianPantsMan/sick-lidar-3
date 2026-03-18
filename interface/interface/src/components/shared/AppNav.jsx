import { NavLink } from "react-router-dom";

const linkClass = ({ isActive }) =>
  `px-4 py-2 rounded-lg text-sm font-semibold transition-colors ${
    isActive ? "bg-[#500000] text-white" : "bg-white text-[#500000] border border-[#500000]"
  }`;

export default function AppNav() {
  return (
    <nav className="flex items-center justify-center gap-3 p-4">
      <NavLink to="/customer" className={linkClass}>
        Customer Interface
      </NavLink>
      <NavLink to="/staff" className={linkClass}>
        Staff Interface
      </NavLink>
    </nav>
  );
}
