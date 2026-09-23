import { mount } from "svelte";
import "./app.css";
import App from "./App.svelte";

// Match the system theme from the very first frame, before settings load.
document.documentElement.dataset.theme = window.matchMedia("(prefers-color-scheme: dark)").matches ? "dark" : "light";

const app = mount(App, { target: document.getElementById("app")! });

// A handle for poking at the viewer from the dev console; never in a build.
if (import.meta.env.DEV) {
  void import("./lib/viewer").then((m) => ((window as unknown as { __xscan: unknown }).__xscan = m));
}
export default app;
