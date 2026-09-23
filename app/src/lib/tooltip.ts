// `use:tip={text}` -- a quiet tooltip that appears after a short hover.

let el: HTMLDivElement | null = null;
let timer = 0;

function ensure(): HTMLDivElement {
  if (el) return el;
  el = document.createElement("div");
  el.className = "xs-tip";
  document.body.appendChild(el);
  const style = document.createElement("style");
  style.textContent = `
  .xs-tip{position:fixed;z-index:9999;max-width:280px;padding:7px 10px;border-radius:9px;font:12px/1.35 var(--font);
    color:var(--text);background:var(--glass-strong);backdrop-filter:blur(24px) saturate(180%);
    border:1px solid var(--glass-border);box-shadow:var(--shadow-sm);pointer-events:none;opacity:0;
    transform:translateY(2px);transition:opacity .16s var(--ease),transform .16s var(--ease);white-space:pre-line}
  .xs-tip.show{opacity:1;transform:none}`;
  document.head.appendChild(style);
  return el;
}

export function tip(node: HTMLElement, text: string | null | undefined) {
  let current = text;
  const show = () => {
    if (!current) return;
    window.clearTimeout(timer);
    timer = window.setTimeout(() => {
      const t = ensure();
      t.textContent = current ?? "";
      const r = node.getBoundingClientRect();
      t.style.left = "0px";
      t.style.top = "0px";
      t.classList.add("show");
      const w = t.offsetWidth, h = t.offsetHeight;
      let x = r.left + r.width / 2 - w / 2;
      let y = r.bottom + 8;
      // Prefer the left side for things hugging the right edge.
      if (r.right > window.innerWidth - 80) {
        x = r.left - w - 10;
        y = r.top + r.height / 2 - h / 2;
      }
      if (y + h > window.innerHeight - 8) y = r.top - h - 8;
      x = Math.max(8, Math.min(window.innerWidth - w - 8, x));
      t.style.left = `${x}px`;
      t.style.top = `${y}px`;
    }, 550);
  };
  const hide = () => {
    window.clearTimeout(timer);
    el?.classList.remove("show");
  };
  node.addEventListener("pointerenter", show);
  node.addEventListener("pointerleave", hide);
  node.addEventListener("pointerdown", hide);
  return {
    update(t: string | null | undefined) {
      current = t;
    },
    destroy() {
      hide();
      node.removeEventListener("pointerenter", show);
      node.removeEventListener("pointerleave", hide);
      node.removeEventListener("pointerdown", hide);
    },
  };
}
