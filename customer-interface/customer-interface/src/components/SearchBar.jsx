import { useState, useEffect } from "react";

export default function SearchBar({ value, onChange, placeholder }) {
    const [local, setLocal] = useState(value);
    useEffect(() => setLocal(value), [value]);

    // debounce
    useEffect(() => {
        const t = setTimeout(() => onChange(local), 250);
        return () => clearTimeout(t);
    }, [local, onChange]);

    return (
        <div className="mb-3">
            <input
                type="search"
                inputMode="search"
                value={local}
                onChange={e => setLocal(e.target.value)}
                placeholder={placeholder}
                className="w-full p-3 rounded-lg border focus:outline-none"
                aria-label="Search products"
            />
        </div>
    );
}