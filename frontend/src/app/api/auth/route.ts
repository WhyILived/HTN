import { NextResponse } from "next/server";
import clientPromise from "@/lib/mongodb";

export async function POST(request: Request) {
  const { email, password, name } = await request.json();
  const url = new URL(request.url);
  const action = url.searchParams.get("action"); // "signup" or "login"

  if (!email || !password) {
    return NextResponse.json({ error: "Email and password required" }, { status: 400 });
  }

  try {
    const client = await clientPromise;
    const db = client.db("htn2025");
    const users = db.collection("users");

    if (action === "signup") {
      const existing = await users.findOne({ email });
      if (existing) {
        return NextResponse.json({ error: "User already exists" }, { status: 400 });
      }
      await users.insertOne({ name, email, password });
      return NextResponse.json({ success: true, user: { name, email } });
    }

    if (action === "login") {
      const user = await users.findOne({ email, password });
      if (!user) return NextResponse.json({ error: "Invalid credentials" }, { status: 401 });
      return NextResponse.json({ success: true, user: { name: user.name, email: user.email } });
    }

    return NextResponse.json({ error: "Invalid action" }, { status: 400 });
  } catch (err) {
    console.error("API /auth error:", err);
    return NextResponse.json({ error: "Server error" }, { status: 500 });
  }
}
