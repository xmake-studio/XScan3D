<script lang="ts">
  import type { Snippet } from "svelte";
  import Icon from "../Icon.svelte";

  let {
    variant = "secondary",
    size = "md",
    icon,
    disabled = false,
    onclick,
    title,
    children,
    wide = false,
  }: {
    variant?: "primary" | "secondary" | "ghost" | "danger" | "tinted";
    size?: "sm" | "md" | "lg";
    icon?: string;
    disabled?: boolean;
    onclick?: (e: MouseEvent) => void;
    title?: string;
    children?: Snippet;
    wide?: boolean;
  } = $props();
</script>

<button class="btn {variant} {size}" class:wide class:icon-only={!children} {disabled} {title} {onclick}>
  {#if icon}<Icon name={icon} size={size === "lg" ? 19 : size === "sm" ? 15 : 17} />{/if}
  {#if children}<span>{@render children()}</span>{/if}
</button>

<style>
  .btn {
    display: inline-flex;
    align-items: center;
    justify-content: center;
    gap: 7px;
    height: 32px;
    padding: 0 14px;
    border-radius: 9px;
    font-weight: 560;
    white-space: nowrap;
    transition: background 0.18s, transform 0.18s var(--spring), opacity 0.2s, box-shadow 0.2s, filter 0.2s;
  }
  .btn:active:not(:disabled) {
    transform: scale(0.97);
  }
  .btn:disabled {
    opacity: 0.4;
  }
  .sm {
    height: 26px;
    padding: 0 10px;
    font-size: 12px;
    border-radius: 7px;
  }
  .lg {
    height: 40px;
    padding: 0 20px;
    font-size: 14px;
    border-radius: 11px;
  }
  .wide {
    width: 100%;
  }
  .icon-only {
    padding: 0;
    width: 32px;
  }
  .sm.icon-only {
    width: 26px;
  }
  .primary {
    background: var(--accent);
    color: var(--on-accent);
    box-shadow: 0 1px 2px rgba(0, 0, 0, 0.15), inset 0 1px 0 rgba(255, 255, 255, 0.15);
  }
  .primary:hover:not(:disabled) {
    background: var(--accent-2);
  }
  .secondary {
    background: var(--fill);
  }
  .secondary:hover:not(:disabled) {
    background: var(--fill-2);
  }
  .tinted {
    background: var(--accent-soft);
    color: var(--accent);
  }
  .tinted:hover:not(:disabled) {
    filter: brightness(1.1);
  }
  .ghost {
    color: var(--text-2);
  }
  .ghost:hover:not(:disabled) {
    background: var(--fill);
    color: var(--text);
  }
  .danger {
    background: color-mix(in srgb, var(--red) 16%, transparent);
    color: var(--red);
  }
  .danger:hover:not(:disabled) {
    background: color-mix(in srgb, var(--red) 24%, transparent);
  }
</style>
