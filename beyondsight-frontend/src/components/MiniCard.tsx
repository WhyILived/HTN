import React from "react";

type MiniCardProps = {
  title: string;
  value: string | number;
  description?: string;
};

const MiniCard: React.FC<MiniCardProps> = ({ title, value, description }) => {
  return (
    <div style={{
      padding: "1rem",
      borderRadius: "0.5rem",
      backgroundColor: "#f3f4f6",
      boxShadow: "0 2px 6px rgba(0,0,0,0.1)",
      minWidth: "150px",
      margin: "0.5rem"
    }}>
      <h3 style={{ fontSize: "1.1rem", marginBottom: "0.5rem", color: "#059669" }}>
        {title}
      </h3>
      <p style={{ fontSize: "1.5rem", fontWeight: 600 }}>{value}</p>
      {description && (
        <p style={{ fontSize: "0.85rem", color: "#4b5563" }}>{description}</p>
      )}
    </div>
  );
};

export default MiniCard;
