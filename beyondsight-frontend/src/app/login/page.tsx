"use client";

import { useState, FormEvent } from "react";
import { useRouter } from "next/navigation";

export default function LoginPage() {
  const [isSignup, setIsSignup] = useState(false);
  const [email, setEmail] = useState("");
  const [name, setName] = useState("");
  const [password, setPassword] = useState("");
  const [message, setMessage] = useState("");
  const router = useRouter();

  const handleSubmit = async (e: FormEvent) => {
    e.preventDefault();
    setMessage("");

    try {
      const res = await fetch(`/api/auth?action=${isSignup ? "signup" : "login"}`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ name, email, password }),
      });

      const data = await res.json();

      if (data.success) {
        localStorage.setItem("user", JSON.stringify(data.user));
        router.push("/profile");
      } else {
        setMessage(data.error);
      }
    } catch (err) {
      console.error(err);
      setMessage("Server error");
    }
  };

  return (
    <div style={{ maxWidth: "400px", margin: "2rem auto", padding: "1rem", border: "1px solid #ccc", borderRadius: "8px" }}>
      <h2>{isSignup ? "Sign Up" : "Login"}</h2>
      <form onSubmit={handleSubmit} style={{ display: "flex", flexDirection: "column", gap: "1rem" }}>
        {isSignup && (
          <label>
            Name:
            <input type="text" value={name} onChange={(e) => setName(e.target.value)} required />
          </label>
        )}
        <label>
          Email:
          <input type="email" value={email} onChange={(e) => setEmail(e.target.value)} required />
        </label>
        <label>
          Password:
          <input type="password" value={password} onChange={(e) => setPassword(e.target.value)} required />
        </label>
        <button type="submit" style={{ padding: "0.5rem", backgroundColor: "#065f46", color: "white", border: "none", borderRadius: "4px" }}>
          {isSignup ? "Sign Up" : "Login"}
        </button>
      </form>
      <p style={{ marginTop: "1rem", cursor: "pointer", color: "blue" }} onClick={() => setIsSignup(!isSignup)}>
        {isSignup ? "Already have an account? Login" : "Don't have an account? Sign Up"}
      </p>
      {message && <p style={{ color: "red" }}>{message}</p>}
    </div>
  );
}
