"use client";

import { useState, useEffect, FormEvent } from "react";
import { useRouter } from "next/navigation";

type User = { name: string; email: string };

export default function ProfilePage() {
  const [originalEmail, setOriginalEmail] = useState<string | null>(null);
  const [user, setUser] = useState<User | null>(null);
  const [message, setMessage] = useState("");
  const [saving, setSaving] = useState(false);
  const router = useRouter();

  useEffect(() => {
    const storedUser = localStorage.getItem("user");
    if (!storedUser) router.push("/login");
    else {
      const u = JSON.parse(storedUser);
      setUser(u);
      setOriginalEmail(u.email);
    }
  }, [router]);

  if (!user) return <p style={{ textAlign: "center", marginTop: "2rem" }}>Loading...</p>;

  const handleSave = async (e: FormEvent) => {
    e.preventDefault();
    setSaving(true);
    setMessage("");

    try {
      const res = await fetch("/api/profile", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ ...user, originalEmail }),
      });
      const data = await res.json();

      if (data.success) {
        setMessage("Profile updated!");
        localStorage.setItem("user", JSON.stringify(user));
        setOriginalEmail(user.email);
      } else {
        setMessage("Failed to update profile");
      }
    } catch (err) {
      console.error(err);
      setMessage("Server error");
    } finally {
      setSaving(false);
    }
  };

  const handleLogout = () => {
    localStorage.removeItem("user");
    router.push("/login");
  };

  return (
    <div
      style={{
        maxWidth: "500px",
        margin: "3rem auto",
        padding: "2rem",
        border: "1px solid #ccc",
        borderRadius: "12px",
        backgroundColor: "#fff",
        color: "#111",
        boxShadow: "0 6px 16px rgba(0,0,0,0.1)",
      }}
    >
      <h2 style={{ textAlign: "center", marginBottom: "2rem" }}>Profile</h2>
      <form
        onSubmit={handleSave}
        style={{ display: "flex", flexDirection: "column", gap: "1.5rem" }}
      >
        <label style={{ display: "flex", flexDirection: "column", fontWeight: 500 }}>
          Name:
          <input
            type="text"
            value={user.name}
            onChange={(e) => setUser({ ...user, name: e.target.value })}
            style={{
              marginTop: "0.5rem",
              padding: "0.75rem",
              borderRadius: "6px",
              border: "1px solid #ccc",
              backgroundColor: "#fafafa",
              color: "#111",
              fontSize: "1rem",
            }}
          />
        </label>
        <label style={{ display: "flex", flexDirection: "column", fontWeight: 500 }}>
          Email:
          <input
            type="email"
            value={user.email}
            readOnly
            style={{
              marginTop: "0.5rem",
              padding: "0.75rem",
              borderRadius: "6px",
              border: "1px solid #ccc",
              backgroundColor: "#f0f0f0",
              color: "#888",
              cursor: "not-allowed",
              fontSize: "1rem",
            }}
          />
        </label>
        <button
          type="submit"
          disabled={saving}
          style={{
            padding: "0.75rem",
            backgroundColor: "#111",
            color: "#fff",
            border: "none",
            borderRadius: "6px",
            fontWeight: 500,
            cursor: saving ? "not-allowed" : "pointer",
            transition: "background-color 0.2s, transform 0.1s",
          }}
          onMouseOver={(e) => !saving && (e.currentTarget.style.backgroundColor = "#333")}
          onMouseOut={(e) => !saving && (e.currentTarget.style.backgroundColor = "#111")}
        >
          {saving ? "Saving..." : "Save"}
        </button>
      </form>
      <button
        onClick={handleLogout}
        style={{
          marginTop: "1.5rem",
          padding: "0.75rem",
          backgroundColor: "#fff",
          color: "#111",
          border: "1px solid #ccc",
          borderRadius: "6px",
          fontWeight: 500,
          width: "100%",
          cursor: "pointer",
          transition: "background-color 0.2s, transform 0.1s",
        }}
        onMouseOver={(e) => (e.currentTarget.style.backgroundColor = "#f5f5f5")}
        onMouseOut={(e) => (e.currentTarget.style.backgroundColor = "#fff")}
      >
        Logout
      </button>
      {message && <p style={{ marginTop: "1.5rem", textAlign: "center" }}>{message}</p>}
    </div>
  );
}
