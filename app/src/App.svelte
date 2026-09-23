<script lang="ts">
  import { onMount } from "svelte";
  import { fade } from "svelte/transition";
  import { api, B } from "./lib/api";
  import ExportSheet from "./lib/components/ExportSheet.svelte";
  import Overlays from "./lib/components/Overlays.svelte";
  import ScanDock from "./lib/components/ScanDock.svelte";
  import SettingsSheet from "./lib/components/SettingsSheet.svelte";
  import Sidebar from "./lib/components/Sidebar.svelte";
  import TitleBar from "./lib/components/TitleBar.svelte";
  import ViewToolbar from "./lib/components/ViewToolbar.svelte";
  import Viewport from "./lib/components/Viewport.svelte";
  import { app, init, openFiles, startScan, stopScan } from "./lib/state.svelte";
  import { view } from "./lib/viewer";

  let error = $state<string | null>(null);

  onMount(() => {
    init()
      .then(() => B().windowShow())
      .catch((e) => {
        error = String(e);
        void api().then((b) => b.windowShow());
      });
  });

  // Theme onto the document and the native window, so resizes never flash.
  $effect(() => {
    document.documentElement.dataset.theme = app.theme;
    if (app.ready) void B().setBackground(app.theme === "dark" ? "#0e0e10" : "#f2f2f5");
  });

  // Keyboard shortcuts that are not tied to one panel.
  function keydown(e: KeyboardEvent) {
    const typing = (e.target as HTMLElement)?.closest("input, textarea, select");
    if (typing || app.settingsOpen || app.exportOpen || app.confirm) return;
    if ((e.ctrlKey || e.metaKey) && (e.code === "Enter" || e.code === "NumpadEnter")) {
      // Start (or stop) a scan without reaching for the mouse.
      e.preventDefault();
      if (app.scan) void stopScan();
      else if (app.countdown === 0) void startScan();
    } else if ((e.ctrlKey || e.metaKey) && e.code === "Comma") {
      e.preventDefault();
      app.settingsOpen = true;
    } else if ((e.ctrlKey || e.metaKey) && e.code === "KeyO") {
      e.preventDefault();
      void openFiles();
    } else if ((e.ctrlKey || e.metaKey) && e.code === "KeyE" && app.selection.length) {
      e.preventDefault();
      app.exportOpen = true;
    } else if (e.code === "KeyF" && !e.ctrlKey && app.settings?.view.camera === "orbit") {
      view.r?.frame(700);
    }
  }
</script>

<svelte:window onkeydown={keydown} oncontextmenu={(e) => {
  if (!(e.target as HTMLElement).closest("input, textarea")) e.preventDefault();
}} />

<main class:ready={app.ready}>
  <Viewport />
  {#if app.ready}
    <TitleBar />
    {#if app.sidebarOpen}<Sidebar />{/if}
    <ViewToolbar />
    <div class="dock-wrap" class:shift={app.sidebarOpen} transition:fade={{ duration: 300 }}>
      <ScanDock />
    </div>
    <Overlays />
    {#if app.settingsOpen}<SettingsSheet />{/if}
    {#if app.exportOpen}<ExportSheet />{/if}
  {:else if error}
    <div class="fatal">{error}</div>
  {/if}
</main>

<style>
  main {
    position: fixed;
    inset: 0;
    overflow: hidden;
    opacity: 0;
    transition: opacity 0.5s var(--ease);
  }
  main.ready {
    opacity: 1;
  }
  .dock-wrap {
    position: absolute;
    left: 50%;
    bottom: 22px;
    translate: -50% 0;
    z-index: 25;
    transition: left 0.35s var(--ease);
  }
  .dock-wrap.shift {
    left: calc(50% + 150px);
  }
  .fatal {
    position: absolute;
    inset: 0;
    display: grid;
    place-items: center;
    color: var(--red);
    padding: 40px;
    text-align: center;
  }
</style>
