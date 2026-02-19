import { useState, useEffect } from "react";
import mock from "../../../customer-interface/mockdb.json"; // or fetch from Firestore

export default function useProducts() {
    const [products, setProducts] = useState([]);
    useEffect(() => {
        // adapt to your data shape; mock.products is an object in your file
        const list = Object.values(mock.products || {}).map((p, i) => ({ id: `prod_${i+1}`, ...p }));
        setProducts(list);
    }, []);
    return products;
}