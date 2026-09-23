<script lang="ts">
  import Icon from "./Icon.svelte";

  type Mode = "off" | "ready" | "countdown" | "busy" | "scanning";
  let { mode, progress = 0, count = 0, onclick, label }: { mode: Mode; progress?: number; count?: number; onclick: () => void; label: string } = $props();
</script>

<button class="shutter {mode}" {onclick} aria-label={label} style="--p:{Math.max(0.001, Math.min(1, progress)) * 100}">
  <svg class="rings" viewBox="0 0 92 92" aria-hidden="true">
    <circle class="track" cx="46" cy="46" r="42.5" />
    <circle class="arc" cx="46" cy="46" r="42.5" pathLength="100" />
  </svg>
  <span class="core">
    {#if mode === "off"}
      <Icon name="plug" size={26} stroke={1.9} />
    {:else if mode === "countdown"}
      {#key count}<span class="count">{count}</span>{/key}
    {:else}
      <!-- A radar screen: rings, crosshair and a sweep that turns while the
           platform does. Hovering during a scan offers the stop button. -->
      <svg class="radar" viewBox="0 0 48 48" aria-hidden="true">
        <defs>
          <linearGradient id="xs-sweep" x1="0" y1="1" x2="0.75" y2="0">
            <stop offset="0" stop-color="currentColor" stop-opacity="0.55" />
            <stop offset="1" stop-color="currentColor" stop-opacity="0" />
          </linearGradient>
        </defs>
        <g class="dish" fill="none" stroke="currentColor" stroke-linecap="round">
          <circle cx="24" cy="24" r="17.5" stroke-width="2" />
          <circle cx="24" cy="24" r="11" stroke-width="1.6" opacity="0.75" />
          <circle cx="24" cy="24" r="4.5" stroke-width="1.6" opacity="0.6" />
          <path d="M24 4.5v6M24 37.5v6M4.5 24h6M37.5 24h6" stroke-width="1.8" opacity="0.8" />
        </g>
        <g class="sweep">
          <path d="M24 24 L24 6.5 A17.5 17.5 0 0 1 39.2 15.3 Z" fill="url(#xs-sweep)" />
          <line x1="24" y1="24" x2="24" y2="6.5" stroke="currentColor" stroke-width="2" stroke-linecap="round" />
        </g>
        <circle class="blip" cx="24" cy="24" r="2.4" fill="currentColor" />
      </svg>
      <span class="stop"></span>
    {/if}
  </span>
</button>

<style>
  .shutter {
    position: relative;
    width: 84px;
    height: 84px;
    flex: none;
    border-radius: 50%;
    display: grid;
    place-items: center;
    transition: transform 0.25s var(--spring);
    -webkit-tap-highlight-color: transparent;
  }
  .shutter:hover {
    transform: scale(1.035);
  }
  .shutter:active {
    transform: scale(0.95);
  }
  .rings {
    position: absolute;
    inset: 0;
    width: 100%;
    height: 100%;
    transform: rotate(-90deg);
    overflow: visible;
  }
  .rings circle {
    fill: none;
    stroke-width: 4;
  }
  .track {
    stroke: var(--fill-2);
    transition: stroke 0.3s;
  }
  .arc {
    stroke: var(--accent);
    stroke-linecap: round;
    stroke-dasharray: 0 100;
    transition: stroke-dasharray 0.4s var(--ease), opacity 0.3s;
    opacity: 0;
  }
  .ready .track {
    stroke: color-mix(in srgb, var(--accent) 38%, transparent);
  }
  .scanning .arc {
    opacity: 1;
    stroke-dasharray: var(--p) 100;
  }
  .busy .arc,
  .countdown .arc {
    opacity: 1;
    stroke-dasharray: 22 100;
    transform-origin: 46px 46px;
    animation: spin 1.1s linear infinite;
  }
  .core {
    position: relative;
    width: 66px;
    height: 66px;
    border-radius: 50%;
    display: grid;
    place-items: center;
    color: #fff;
    background: radial-gradient(120% 120% at 30% 20%, #4aa3ff 0%, var(--accent) 45%, #0759d6 100%);
    box-shadow: 0 6px 20px color-mix(in srgb, var(--accent) 45%, transparent), inset 0 1px 0 rgba(255, 255, 255, 0.35),
      inset 0 -6px 14px rgba(0, 0, 0, 0.18);
    transition: background 0.35s var(--ease), box-shadow 0.35s var(--ease), width 0.35s var(--ease), height 0.35s var(--ease);
  }
  .ready:hover .core {
    box-shadow: 0 8px 30px color-mix(in srgb, var(--accent) 65%, transparent), inset 0 1px 0 rgba(255, 255, 255, 0.35),
      inset 0 -6px 14px rgba(0, 0, 0, 0.18);
  }
  .off .core {
    background: var(--fill-2);
    color: var(--text-2);
    box-shadow: inset 0 1px 0 var(--glass-hi);
  }
  .off .track {
    animation: breathe 2.4s ease-in-out infinite;
  }
  .scanning .core,
  .busy .core {
    background: radial-gradient(120% 120% at 30% 20%, #3a93f5 0%, #0b6ad6 50%, #064aa8 100%);
  }
  .radar {
    position: absolute;
    width: 42px;
    height: 42px;
    transition: opacity 0.22s var(--ease), transform 0.22s var(--ease);
  }
  .sweep {
    transform-origin: 24px 24px;
    opacity: 0;
    transition: opacity 0.3s;
  }
  /* The sweep turns while a scan runs, and gives a hint of life on hover. */
  .scanning .sweep,
  .busy .sweep {
    opacity: 1;
    animation: spin 2.6s linear infinite;
  }
  .ready:hover .sweep {
    opacity: 0.9;
    animation: spin 3.6s linear infinite;
  }
  .blip {
    opacity: 0.9;
  }
  .scanning .blip {
    animation: pulse-soft 1.4s ease-in-out infinite;
  }
  .stop {
    position: absolute;
    width: 22px;
    height: 22px;
    border-radius: 6px;
    background: var(--red);
    box-shadow: 0 2px 12px color-mix(in srgb, var(--red) 55%, transparent);
    opacity: 0;
    transform: scale(0.6);
    transition: opacity 0.2s var(--ease), transform 0.25s var(--spring);
    pointer-events: none;
  }
  /* Hovering a running scan offers to stop it. */
  .scanning:hover .stop,
  .busy:hover .stop {
    opacity: 1;
    transform: scale(1);
  }
  .scanning:hover .radar,
  .busy:hover .radar {
    opacity: 0;
    transform: scale(0.82);
  }
  .count {
    font-size: 30px;
    font-weight: 700;
    font-variant-numeric: tabular-nums;
    animation: pop 0.45s var(--spring);
  }
  @keyframes pop {
    from {
      transform: scale(0.4);
      opacity: 0;
    }
  }
  @keyframes breathe {
    0%,
    100% {
      stroke: var(--fill-2);
    }
    50% {
      stroke: var(--fill-3);
    }
  }
</style>
