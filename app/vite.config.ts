import { defineConfig, type Plugin } from "vite";
import { svelte } from "@sveltejs/vite-plugin-svelte";
import fs from "node:fs";
import path from "node:path";

// UI development in a plain browser: serve exported captures from
// ./dev-assets at /dev-assets (dev server only, never bundled).
function devAssets(): Plugin {
  const dir = path.resolve(__dirname, "dev-assets");
  return {
    name: "xscan-dev-assets",
    apply: "serve",
    configureServer(server) {
      server.middlewares.use("/dev-assets", (req, res, next) => {
        const file = path.join(dir, decodeURIComponent((req.url ?? "/").split("?")[0]));
        if (!file.startsWith(dir) || !fs.existsSync(file) || fs.statSync(file).isDirectory()) return next();
        res.setHeader("Content-Type", file.endsWith(".json") ? "application/json" : "application/octet-stream");
        fs.createReadStream(file).pipe(res);
      });
    },
  };
}

// Tauri serves the built assets itself; the dev server only needs a fixed port
// it can find, and must not clear the terminal Tauri is logging to.
export default defineConfig({
  plugins: [svelte(), devAssets()],
  clearScreen: false,
  server: {
    port: 1420,
    strictPort: true,
    watch: { ignored: ["**/src-tauri/**", "**/dev-assets/**"] },
  },
  build: {
    target: "es2022",
    chunkSizeWarningLimit: 2000,
  },
});
