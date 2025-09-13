import "../styles/globals.css";
import Link from "next/link";
import styles from "../styles/Layout.module.css";

export const metadata = {
  title: "Dashboard",
  description: "Dashboard App",
};

export default function RootLayout({
  children,
}: {
  children: React.ReactNode;
}) {
  return (
    <html lang="en">
      <body className={styles.body}>
        <header className={styles.navbar}>
          <h1 className={styles.logo}>HTN2025 Dashboard</h1>
          <nav className={styles.navLinks}>
            <Link href="/" className={styles.link}>
              Home
            </Link>
            <Link href="/profile" className={styles.link}>
              Profile
            </Link>
          </nav>
        </header>
        <main className={styles.main}>{children}</main>
      </body>
    </html>
  );
}
