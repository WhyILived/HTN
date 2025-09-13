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
            setOriginalEmail(u.email); // track original email
        }
    }, [router]);

    if (!user) return <p>Loading...</p>;

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
                // Save updated user to localStorage
                localStorage.setItem("user", JSON.stringify(user));
                setOriginalEmail(user.email); // update original email
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
        <div style={{ maxWidth: "500px", margin: "2rem auto", padding: "1rem", border: "1px solid #ccc", borderRadius: "8px" }}>
            <h2>Profile</h2>
            <form onSubmit={handleSave} style={{ display: "flex", flexDirection: "column", gap: "1rem" }}>
                <label>
                    Name:
                    <input type="text" value={user.name} onChange={(e) => setUser({ ...user, name: e.target.value })} />
                </label>
                <label>
                    Email:
                    <input
                        type="email"
                        value={user.email}
                        readOnly
                        style={{ backgroundColor: "#f0f0f0", cursor: "not-allowed" }}
                    />
                </label>
                <button type="submit" disabled={saving} style={{ padding: "0.5rem", backgroundColor: "#065f46", color: "white", border: "none", borderRadius: "4px" }}>
                    {saving ? "Saving..." : "Save"}
                </button>
            </form>
            <button onClick={handleLogout} style={{ marginTop: "1rem", padding: "0.5rem", backgroundColor: "#b91c1c", color: "white", border: "none", borderRadius: "4px" }}>
                Logout
            </button>
            {message && <p style={{ marginTop: "1rem" }}>{message}</p>}
        </div>
    );
}
