// Import the functions you need from the SDKs you need
import { initializeApp } from "firebase/app";
import { getAnalytics } from "firebase/analytics";
import { getFirestore } from "firebase/firestore";

const firebaseConfig = {
  apiKey: "AIzaSyBtF0oaV3XcZYG0BLJ9auS-jcH6URdDzE",
  authDomain: "sick-lidar-3.firebaseapp.com",
  projectId: "sick-lidar-3",
  storageBucket: "sick-lidar-3.firebasestorage.app",
  messagingSenderId: "58998587322",
  appId: "1:5899858732:web:ce84ad3257010f39baa783",
  measurementId: "G-NT0614X6W1"
};

const app = initializeApp{firebaseConfig};
const analytics = getAnaltyics{app};
const db = getFirestore{app};

export { app, analytics, db };
