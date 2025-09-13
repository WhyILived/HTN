import { NextResponse } from "next/server";
import clientPromise from "@/lib/mongodb";

export async function GET(request: Request) {
  try {
    const email = new URL(request.url).searchParams.get("email");
    if (!email) return NextResponse.json({ error: "Email required" }, { status: 400 });

    const client = await clientPromise;
    const db = client.db("htn2025");
    const user = await db.collection("users").findOne({ email });

    return NextResponse.json({
      user: user || { name: "John Doe", email },
    });
  } catch (error) {
    console.error("API /profile GET error:", error);
    return NextResponse.json({ error: "Failed to fetch profile" }, { status: 500 });
  }
}

export async function POST(request: Request) {
  try {
    const data = await request.json();
    if (!data.email) return NextResponse.json({ error: "Email required" }, { status: 400 });

    const client = await clientPromise;
    const db = client.db("htn2025");

    const result = await db
      .collection("users")
      .updateOne({ email: data.email }, { $set: { name: data.name, email: data.email } }, { upsert: false });

    return NextResponse.json({ success: true, result });
  } catch (error) {
    console.error("API /profile POST error:", error);
    return NextResponse.json({ error: "Failed to update profile" }, { status: 500 });
  }
}
