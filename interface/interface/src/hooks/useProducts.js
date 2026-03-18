import { useState, useEffect } from "react";
import { collection, onSnapshot } from "firebase/firestore";
import { db } from "../firebase/config";

export default function useProducts() {
    const [products, setProducts] = useState([]);

    useEffect(() => {
        // Set up real-time listener for products collection
        const unsubscribe = onSnapshot(
            collection(db, "products"),
            (snapshot) => {
                const productsList = [];
                snapshot.forEach((doc) => {
                    productsList.push({
                        id: doc.id,
                        ...doc.data(),
                    });
                });
                setProducts(productsList);
            },
            (err) => {
                console.error("Error fetching products:", err);
            }
        );

        // Cleanup listener on unmount
        return () => unsubscribe();
    }, []);

    return products;
}