import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

// Tauri loads the dev server at a fixed port (see tauri.conf.json devUrl).
export default defineConfig({
  plugins: [react()],
  clearScreen: false,
  server: {
    port: 1420,
    strictPort: true,
    watch: {
      // Tauri watches Rust sources; Cargo's build outputs can be locked on Windows.
      ignored: ["**/src-tauri/**"],
    },
  },
  build: {
    target: "es2021",
    outDir: "dist",
    emptyOutDir: true,
    sourcemap: false,
  },
});
