"use client";

import React, { useEffect, useState } from "react";
import MiniCard from "@/components/MiniCard";

const HomePage: React.FC = () => {
  const [status, setStatus] = useState<string>("Loading...");

  useEffect(() => {
    async function fetchStatus() {
      try {
        const res = await fetch("http://127.0.0.1:5000/all_messages");
        const data = await res.json();

        if (data.length === 0) {
          setStatus("No messages yet");
          return;
        }

        const lastMessages = data[data.length - 1].messages;
        setStatus(lastMessages.join("\n"));
      } catch (err) {
        console.error("Error fetching messages:", err);
        setStatus("Error fetching messages");
      }
    }

    fetchStatus();
    const interval = setInterval(fetchStatus, 2000);
    return () => clearInterval(interval);
  }, []);

  const stats = [
    {
      title: "Braille Input",
      value: status,
      description: "Latest sentence from Python backend",
    },
    {
      title: "Summarize Input",
      value:
      "The conversation was about how they had a great time at Hack the North as well as how it is a great day outside.",
      description: "Latest Conversation Summary (Hardcoded due to no access to Genesys Summarizer API)",
    },
  ];

  return (
    <div
      style={{
        maxWidth: "1200px",
        margin: "3rem auto",
        padding: "2rem",
        backgroundColor: "rgba(255, 255, 255, 0)", // transparent
        color: "#111",
        borderRadius: "12px",
      }}
    >
      <h1 style={{ fontSize: "2rem", marginBottom: "2rem", textAlign: "center" }}>
        Dashboard
      </h1>
      <div
        style={{
          display: "flex",
          flexWrap: "wrap",
          gap: "1.5rem",
          justifyContent: "center",
        }}
      >
        {stats.map((stat, idx) => (
          <div key={idx} style={{ flex: "1 1 250px" }}>
            <MiniCard
              title={stat.title}
              value={stat.value}
              description={stat.description}
            />
          </div>
        ))}
      </div>
    </div>
  );
};

export default HomePage;
