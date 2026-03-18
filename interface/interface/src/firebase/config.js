import { initializeApp } from "firebase/app";
import { getFirestore } from "firebase/firestore";

// Your Firebase configuration - using the provided credentials
const firebaseConfig = {
  apiKey: "AIzaSyBtF0oaV3XcZYGOm8LJ9auS-jcH6URdDzE",
  authDomain: "sick-lidar-3.firebaseapp.com",
  projectId: "sick-lidar-3",
  storageBucket: "sick-lidar-3.firebasestorage.app",
  messagingSenderId: "58998587322",
  appId: "1:58998587322:web:2941ce60b9b0be9ebaa783",
  measurementId: "G-0QTE197F4W"
};

// Initialize Firebase
const app = initializeApp(firebaseConfig);

// Initialize Firestore
export const db = getFirestore(app);

export default app;
