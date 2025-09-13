import React from "react";
import MiniCard from "@/components/MiniCard";

const HomePage: React.FC = () => {
  const stats = [
    { title: "Users", value: 124, description: "Active users today" },
    { title: "Projects", value: 8, description: "Ongoing projects" },
    { title: "Revenue", value: "$1,240", description: "This month" },
  ];

  return (
    <div style={{ padding: "2rem" }}>
      <h1 style={{ fontSize: "2rem", marginBottom: "1rem", color: "#059669" }}>
        Dashboard
      </h1>
      <div style={{ display: "flex", flexWrap: "wrap" }}>
        {stats.map((stat, idx) => (
          <MiniCard
            key={idx}
            title={stat.title}
            value={stat.value}
            description={stat.description}
          />
        ))}
      </div>
    </div>
  );
};

export default HomePage;
