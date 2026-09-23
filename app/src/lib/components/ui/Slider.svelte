<script lang="ts">
  // A range input with an Apple-style track, filled up to the thumb.
  let {
    value = $bindable(0),
    min = 0,
    max = 1,
    step = 0.001,
    oninput,
    onchange,
    disabled = false,
    label = "",
  }: {
    value?: number;
    min?: number;
    max?: number;
    step?: number;
    oninput?: (v: number) => void;
    onchange?: (v: number) => void;
    disabled?: boolean;
    label?: string;
  } = $props();

  let pct = $derived(((value - min) / (max - min || 1)) * 100);
</script>

<input
  class="slider"
  type="range"
  aria-label={label}
  {min}
  {max}
  {step}
  {disabled}
  bind:value
  style="--p:{pct}%"
  oninput={() => oninput?.(value)}
  onchange={() => onchange?.(value)}
/>

<style>
  .slider {
    -webkit-appearance: none;
    appearance: none;
    width: 100%;
    height: 22px;
    background: transparent;
    margin: 0;
    cursor: pointer;
  }
  .slider:disabled {
    opacity: 0.4;
    cursor: default;
  }
  .slider::-webkit-slider-runnable-track {
    height: 5px;
    border-radius: 999px;
    background: linear-gradient(90deg, var(--accent) 0 var(--p), var(--fill-2) var(--p) 100%);
  }
  .slider::-webkit-slider-thumb {
    -webkit-appearance: none;
    width: 21px;
    height: 21px;
    margin-top: -8px;
    border-radius: 50%;
    background: #fff;
    box-shadow: 0 1px 4px rgba(0, 0, 0, 0.3), 0 0 0 0.5px rgba(0, 0, 0, 0.08);
    transition: transform 0.18s var(--spring);
  }
  .slider:active::-webkit-slider-thumb {
    transform: scale(1.12);
  }
  .slider:focus-visible {
    outline: none;
  }
  .slider:focus-visible::-webkit-slider-thumb {
    box-shadow: 0 0 0 3px var(--accent-soft), 0 1px 4px rgba(0, 0, 0, 0.3);
  }
</style>
