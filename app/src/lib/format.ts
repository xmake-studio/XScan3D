// Numbers, durations and dates in the user's language.

import { i18n, locale, t } from "./i18n.svelte";

export function compact(n: number): string {
  if (!isFinite(n)) return "—";
  const ru = i18n.lang === "ru";
  const nf = (v: number, d: number) => v.toLocaleString(locale(), { maximumFractionDigits: d, minimumFractionDigits: 0 });
  if (n >= 1e6) return `${nf(n / 1e6, n >= 1e7 ? 0 : 1)}${ru ? " " : ""}${t("units.m")}`;
  if (n >= 1e4) return `${nf(n / 1e3, 0)}${ru ? " " : ""}${t("units.k")}`;
  return nf(n, 0);
}

export function duration(sec: number): string {
  sec = Math.max(0, Math.round(sec));
  if (sec < 60) return `${sec} ${t("units.sec")}`;
  const m = Math.floor(sec / 60), s = sec % 60;
  if (m < 60) return s && m < 10 ? `${m} ${t("units.min")} ${s} ${t("units.sec")}` : `${m} ${t("units.min")}`;
  const h = Math.floor(m / 60), mm = m % 60;
  return mm ? `${h} ${t("units.hour")} ${mm} ${t("units.min")}` : `${h} ${t("units.hour")}`;
}

/** m:ss / h:mm:ss for a countdown. */
export function clock(sec: number): string {
  sec = Math.max(0, Math.round(sec));
  const h = Math.floor(sec / 3600), m = Math.floor((sec % 3600) / 60), s = sec % 60;
  const p = (v: number) => String(v).padStart(2, "0");
  return h ? `${h}:${p(m)}:${p(s)}` : `${m}:${p(s)}`;
}

export function dateTime(iso: string): string {
  const d = new Date(iso);
  if (isNaN(d.getTime())) return iso;
  const now = new Date();
  const sameYear = d.getFullYear() === now.getFullYear();
  return d.toLocaleString(locale(), {
    day: "numeric",
    month: "short",
    ...(sameYear ? {} : { year: "numeric" }),
    hour: "2-digit",
    minute: "2-digit",
  });
}

export function fixed(v: number, digits = 1): string {
  return v.toLocaleString(locale(), { maximumFractionDigits: digits, minimumFractionDigits: digits });
}

export function bytes(n: number): string {
  if (n >= 1 << 30) return `${fixed(n / (1 << 30), 1)} GB`;
  if (n >= 1 << 20) return `${fixed(n / (1 << 20), 1)} MB`;
  return `${Math.round(n / 1024)} KB`;
}
