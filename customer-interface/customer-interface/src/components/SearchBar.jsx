import { useState, useEffect, useRef } from "react";
import Keyboard from "react-simple-keyboard";
import "react-simple-keyboard/build/css/index.css";
import "./SearchBar.css";

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
  const [showKeyboard, setShowKeyboard] = useState(false);
  const keyboard = useRef();
  const inputRef = useRef();
  const keyboardContainerRef = useRef();

  useEffect(() => setLocal(value), [value]);

  // Sync keyboard with input value when keyboard is shown or value changes
  useEffect(() => {
    if (showKeyboard && keyboard.current) {
      keyboard.current.setInput(local);
    }
  }, [showKeyboard, local]);

  // Handle click outside to close keyboard
  useEffect(() => {
    const handleClickOutside = (event) => {
      if (
        keyboardContainerRef.current &&
        !keyboardContainerRef.current.contains(event.target) &&
        !inputRef.current?.contains(event.target)
      ) {
        setShowKeyboard(false);
      }
    };

    if (showKeyboard) {
      document.addEventListener("mousedown", handleClickOutside);
      return () => document.removeEventListener("mousedown", handleClickOutside);
    }
  }, [showKeyboard]);

  // debounce search + optionally filter products and emit results
  useEffect(() => {
    const t = setTimeout(() => {
      onChange(local); // keep existing contract (query string)

      if (Array.isArray(products) && products.length > 0) {
        const q = local.trim().toLowerCase();
        const filtered = products.filter((p) => {
          const inCategory = category === "All Products" || p.productCategory === category;
          const matchesQuery = q === "" || (p.productName || "").toLowerCase().includes(q);
          return inCategory && matchesQuery;
        });
        onResults(filtered);
      }
    }, 250);

    return () => clearTimeout(t);
  }, [local, onChange, products, onResults, category]);

  const handleKeyboardChange = (input) => {
    setLocal(input);
  };

  const onKeyPress = (button) => {
    if (button === "{enter}") {
      setShowKeyboard(false);
      inputRef.current?.blur();
    }
  };

  const handleInputClick = () => {
    setShowKeyboard(true);
    // Sync keyboard with current input immediately
    if (keyboard.current) {
      keyboard.current.setInput(local);
    }
  };

  const handleInputChange = (e) => {
    const input = e.target.value;
    setLocal(input);
    if (keyboard.current) {
      keyboard.current.setInput(input);
    }
  };

  return (
    <div className="w-full relative">
      <input
        ref={inputRef}
        type="text"
        value={local}
        onChange={handleInputChange}
        onClick={handleInputClick}
        onFocus={handleInputClick}
        placeholder={placeholder}
        className="w-full max-w-7xl p-4 rounded-xl border border-gray-300 bg-white shadow-lg focus:outline-none focus:ring-2 focus:ring-[#500000] text-lg text-black placeholder-gray-500"
        aria-label="Search products"
      />
      {showKeyboard && (
        <div ref={keyboardContainerRef} className="fixed inset-x-0 bottom-0 z-50 w-screen bg-white shadow-2xl p-6 border-t-4 border-gray-200">
          <Keyboard
            keyboardRef={(r) => (keyboard.current = r)}
            onChange={handleKeyboardChange}
            onKeyPress={onKeyPress}
            layout={{
              default: [
                "q w e r t y u i o p",
                "a s d f g h j k l",
                "z x c v b n m {bksp}",
                "{space} {enter}",
              ],
            }}
            display={{
              "{bksp}": "⌫ Delete",
              "{enter}": "Enter ↵",
              "{space}": "Space",
            }}
            theme="hg-theme-default hg-layout-default"
            buttonTheme={[
              {
                class: "hg-red",
                buttons: "{bksp}",
              },
              {
                class: "hg-highlight",
                buttons: "{enter}",
              },
            ]}
          />
        </div>
      )}
    </div>
  );
}